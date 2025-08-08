#include "arm_math_types.h"
#include "zephyr/fatal_types.h"
#include "zephyr/sys/__assert.h"
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_audio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/display.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dmic_sample, LOG_LEVEL_DBG);

#include <hal/nrf_pdm.h>

#include <dsp/statistics_functions.h>

#include <melvis.h>

#define MAX_SAMPLE_RATE  16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT     10

/* Size of a block for 100 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
	(BYTES_PER_SAMPLE * (_sample_rate / 1000) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
#define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 1)
#define BLOCK_COUNT      16

K_MEM_SLAB_DEFINE_STATIC(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);

NET_BUF_POOL_DEFINE(audio_buf_pool, 4, MAX_BLOCK_SIZE, 0, NULL);

static const struct device *const usb_audio_mic = DEVICE_DT_GET(DT_NODELABEL(mic_dev));
static const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));

static int16_t  mic_volume = 100;
static bool     mic_muted  = false;
static int      data_dropped = 0;
static int      data_received = 0;
static int      data_sent = 0;
static int16_t  data_min = 0;
static int16_t  data_max = 0;
static int16_t  data_avg = 0;
static size_t   data_len = 0;
static int16_t  data_fst = 0;

K_MSGQ_DEFINE(audio_data_msgq, sizeof(struct net_buf*), 1, 4);
K_MSGQ_DEFINE(audio_process_msgq, sizeof(struct net_buf*), 8, 4);

K_SEM_DEFINE(read_pdm_mic_start_sem, 0, 1);

void stats_buffer(void *buffer, size_t len, int16_t *min, int16_t *max, int16_t *avg);

void read_pdm_mic_thread_func() {
    k_sem_take(&read_pdm_mic_start_sem, K_FOREVER);
    LOG_INF("Starting read PDM mic thread");
    int ret = 0;

    ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
    if (ret != 0) {
        LOG_ERR("Failed to start DMIC: %d", ret);
        return;
    }

    void *buffer = NULL;
    struct net_buf *data = NULL;
    size_t buffer_size = 0;
    while (true) {
        ret = dmic_read(dmic_dev, 0, &buffer, &buffer_size, READ_TIMEOUT);
        if (ret < 0) {
            LOG_ERR("Failed to read audio data: %d", ret);
            break;
        }
        // stats
        data_len = buffer_size;
        data_fst = UNALIGNED_GET((int16_t *)buffer);
        stats_buffer(buffer, buffer_size, &data_min, &data_max, &data_avg);
        data_received++;
        // send usb mic data
        data = net_buf_alloc(&audio_buf_pool, K_NO_WAIT);
        if (!data) {
            LOG_ERR("Failed to allocate audio buffer");
            break;
        }
        net_buf_add_mem(data, buffer, buffer_size);

        while (k_msgq_put(&audio_data_msgq, &data, K_NO_WAIT) != 0) {
            struct net_buf *old = NULL;
            k_msgq_get(&audio_data_msgq, &old, K_NO_WAIT);
            if (old) {
                net_buf_unref(old);
                data_dropped++;
            }
        }
        // send mel spec processing data
        melvis_on_audio_data(buffer, buffer_size / sizeof(int16_t));
        k_mem_slab_free(&mem_slab, buffer);
    }
}

K_THREAD_DEFINE(read_pdm_mic_thread, 1024, read_pdm_mic_thread_func, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(7), 0, 0);

void print_stats_thread_func() {
    while (true) {
        k_sleep(K_SECONDS(10));
        LOG_INF("Audio data, dropped: %d, received: %d,"
                " sent: %d, min: %d, max: %d, avg: %d, len: %d, fst: %d",
                data_dropped, data_received,
                data_sent, data_min, data_max, data_avg, data_len, data_fst);
    }
}

K_THREAD_DEFINE(print_stats_thread, 512, print_stats_thread_func, NULL, NULL, NULL,
                    K_PRIO_PREEMPT(8), 0, 0);

int init_dmic(const struct device *const dev) {
    int ret = 0;
    // Initialize the digital microphone
    if (!device_is_ready(dmic_dev)) {
        LOG_ERR("%s is not ready", dmic_dev->name);
        return -1;
    }

	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab  = &mem_slab,
	};
	struct dmic_cfg cfg = {
		.io = {
			/* These fields can be used to limit the PDM clock
			 * configurations that the driver is allowed to use
			 * to those supported by the microphone.
			 */
			.min_pdm_clk_freq = 1100000,
			.max_pdm_clk_freq = 3500000,
			.min_pdm_clk_dc   = 40,
			.max_pdm_clk_dc   = 60,
		},
		.streams = &stream,
		.channel = { },
	};

	cfg.channel.req_num_chan = 1;
	cfg.channel.req_num_streams = 1;
    // according to spec, MIC L/R is connected to GND
    // sample data on rising edge
    // and this configures nrf pdm as such
	cfg.channel.req_chan_map_lo =
		dmic_build_channel_map(0, 0, PDM_CHAN_RIGHT);
	cfg.streams[0].pcm_rate   = MAX_SAMPLE_RATE;
	cfg.streams[0].pcm_width  = 16;
	cfg.streams[0].block_size =
		BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);
    if (ret != 0) {
        LOG_ERR("Failed to enable mic regulator: %d", ret);
        return ret;
    }
    ret = dmic_configure(dmic_dev, &cfg);
    if (ret != 0) {
        LOG_ERR("Failed to configure DMIC: %d", ret);
        return ret;
    }
    nrf_pdm_gain_set(NRF_PDM0, 0x50, 0x50);
    k_sem_give(&read_pdm_mic_start_sem);
    LOG_INF("DMIC dev initialized");
    return ret;
}

static void feature_update(const struct device *dev, const struct usb_audio_fu_evt *evt);

void send_audio_data(const struct device *dev);

static const struct usb_audio_ops mic_ops = {
    .data_request_cb = send_audio_data,
    .feature_update_cb = feature_update,
};

int init_usb_audio_mic(const struct device *const dev) {
    if (!device_is_ready(usb_audio_mic)) {
        LOG_ERR("USB audio microphone device not ready");
        return -1;
    }

    usb_audio_register(usb_audio_mic, &mic_ops);

    LOG_INF("USB audio microphone initialized successfully");
    return 0;
}

static void feature_update(const struct device *dev,
			   const struct usb_audio_fu_evt *evt)
{
	LOG_DBG("Control selector %d for channel %d updated",
		evt->cs, evt->channel);
	switch (evt->cs) {
	case USB_AUDIO_FU_MUTE_CONTROL:
        mic_muted = UNALIGNED_GET((bool *)evt->val);
        LOG_INF("set mute: %s", mic_muted ? "true" : "false");
		break;
	case USB_AUDIO_FU_VOLUME_CONTROL:
		mic_volume = UNALIGNED_GET((int16_t *)evt->val);
		LOG_INF("set volume: %d", mic_volume);
		break;
	default:
		break;
	}
}

void send_audio_data(const struct device *dev) {
    struct net_buf *buf = NULL;
    int ret = 0;
    // size_t frame_size = usb_audio_get_in_frame_size(usb_audio_mic);
    k_msgq_get(&audio_data_msgq, &buf, K_FOREVER);
    ret = usb_audio_send(usb_audio_mic, buf, buf->len);
    if (ret < 0) {
        LOG_ERR("Failed to send audio data: %d", ret);
        goto cleanup;
    }
    data_sent++;
cleanup:
    if (buf) {
        net_buf_unref(buf);
    }
}

void stats_buffer(void *buffer, size_t len, int16_t *min, int16_t *max, int16_t *avg) {
    int16_t *p = (int16_t *)buffer;
    size_t n = len / sizeof(int16_t);
    size_t _idx;
    arm_max_q15(p, n, max, &_idx);
    arm_min_q15(p, n, min, &_idx);
    arm_mean_q15(p, n, avg);
}

int main() {
    LOG_INF("nrf USB MIC!");
    int ret;

    ret = init_dmic(dmic_dev);
    if (ret < 0) {
        LOG_ERR("Failed to initialize DMIC: %d", ret);
        return ret;
    }

    ret = init_usb_audio_mic(usb_audio_mic);
    if (ret < 0) {
        LOG_ERR("Failed to initialize USB audio mic: %d", ret);
        return ret;
    }

    ret = usb_enable(NULL);
    if (ret != 0) {
        LOG_ERR("Failed to enable USB: %d", ret);
        return ret;
    }

    LOG_INF("Initialization completed");
    return 0;
}
