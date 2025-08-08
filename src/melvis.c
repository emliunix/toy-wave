/**
 * simple shapes drawing
 */

#include <stdint.h>

#include <sys/_stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(melvis);

#include <zephyr/drivers/display.h>

#include <melspec.h>
#include <melvis.h>

static const struct device *const ssd1306_dev = DEVICE_DT_GET(DT_NODELABEL(ssd1306));

RING_BUF_ITEM_DECLARE(mel_ringbuf, (MELSPEC_FFT_SIZE + 256) * sizeof(int16_t));
K_MUTEX_DEFINE(mel_ringbuf_mutex);
K_SEM_DEFINE(mel_ringbuf_sem, 0, 1);
K_MUTEX_DEFINE(mel_pixbuf_mutex);
K_SEM_DEFINE(mel_pixbuf_sem, 0, 1);

static melspec_t mel;
static float32_t mel_buf[MELSPEC_FFT_SIZE];
static uint8_t mel_pixbuf[8 * 128];
static const uint8_t pix_lut[] = { 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF };

void melvis_on_audio_data(int16_t *data, size_t size) {
  k_mutex_lock(&mel_ringbuf_mutex, K_FOREVER);
  int n_discard = size * sizeof(int16_t) - ring_buf_space_get(&mel_ringbuf);
  if (n_discard > 0) {
    // drop data, only keep the latest 1024 data points
    ring_buf_get(&mel_ringbuf, NULL, ring_buf_size_get(&mel_ringbuf) - MELSPEC_FFT_SIZE * sizeof(int16_t));
  }
  int n_put;
  while (true) {
    n_put = ring_buf_put(&mel_ringbuf, (uint8_t *)data, size * sizeof(int16_t));
    if (n_put == size * sizeof(int16_t)) {
      break;
    }
    data += n_put / sizeof(int16_t);
    size -= n_put / sizeof(int16_t);
  }
  k_mutex_unlock(&mel_ringbuf_mutex);
  k_sem_give(&mel_ringbuf_sem);
}

static void render(uint8_t* pixbuf, int* bars) {
  memset(pixbuf, 0, 8 * 128);
  for (int i = 0; i < 32; i++) {
    int b = bars[i];
    // full byte
    for (int j = 0; j < b / 8; j++) {
      size_t base_idx = (8 - 1 - j) * 128;
      /* LOG_DBG("bar %d, row %d, base_idx %zu", i, j, base_idx); */
      for (int x = i * 4; x < (i + 1) * 4; x++) {
         size_t idx = base_idx + x;
        __ASSERT_NO_MSG(idx >= 0 && idx < 8 * 128);
        pixbuf[idx] = 0xFF;
      }
    }
    if (b % 8 > 0) {
      // last row
      size_t base_idx = (8 - 1 - b / 8) * 128;
      /* LOG_DBG("bar %d, row %d, base_idx %zu", i, b / 8, base_idx); */
      for (int x = i * 4; x < (i + 1) * 4; x++) {
        size_t idx = base_idx + x;
        __ASSERT_NO_MSG(idx >= 0 && idx < 8 * 128);
        pixbuf[idx] = pix_lut[b % 8];
      }
    }
  }
}

static uint16_t audio_render_buf[MELSPEC_FFT_SIZE];

void melvis_render_thread_func() {
  if (melspec_init(&mel, 1.0f, 16000, mel_buf, 1024)) {
    LOG_ERR("Failed to initialize mel spectrogram\n");
    return;
  }
  while (true) {
    k_sem_take(&mel_ringbuf_sem, K_FOREVER);
    k_mutex_lock(&mel_ringbuf_mutex, K_FOREVER);
    if (ring_buf_size_get(&mel_ringbuf) < 1024) {
      k_mutex_unlock(&mel_ringbuf_mutex);
      continue; // not enough data
    }
    /* LOG_INF("Filling mel buffer, ringbuf size: %d", */
    /*         ring_buf_size_get(&mel_ringbuf)); */
    // move to mel buf
    melspec_shift(&mel, melspec_get_size(&mel));
    ring_buf_get(&mel_ringbuf, (uint8_t *)audio_render_buf, 512 * sizeof(int16_t));
    ring_buf_peek(&mel_ringbuf, (uint8_t *)(audio_render_buf + 512), (MELSPEC_FFT_SIZE - 512) * sizeof(int16_t));
    melspec_put(&mel, audio_render_buf, MELSPEC_FFT_SIZE);
    k_mutex_unlock(&mel_ringbuf_mutex);
    /* LOG_INF("Rendering mel spectrogram, size: %d", */
    /*         melspec_get_size(&mel)); */
    // process
    melspec_process(&mel);
    melspec_get_mel_spectrum(&mel);
    float32_t* spectrum = melspec_get_mel_spectrum(&mel);
    // calc bars
    int bars[32];
    for (int i = 0; i < 32; i++) {
      // scale to 0-64
      bars[i] = (int)(spectrum[i]);
      if (bars[i] > 64) {
        bars[i] = 64;
      }
      if (bars[i] < 0) {
        bars[i] = 0;
      }
    }
    k_mutex_lock(&mel_pixbuf_mutex, K_FOREVER);
    render(mel_pixbuf, bars);
    k_mutex_unlock(&mel_pixbuf_mutex);
    k_sem_give(&mel_pixbuf_sem);
  }
}

K_THREAD_DEFINE(melvis_render_thread, 1024,
                melvis_render_thread_func, NULL, NULL, NULL,
                K_PRIO_PREEMPT(12), 0, 0);

static void draw(uint8_t* pixbuf) {
  struct display_buffer_descriptor desc = {
    .buf_size = 128,
    .width = 128,
    .height = 8,
    .pitch = 128,
  };
  for (int j = 0; j < 8; j++) {
    display_write(ssd1306_dev, 0, j * 8, &desc, &pixbuf[j*128]);
    k_yield();
  }
}

void melvis_draw_thread_func() {
  if (!device_is_ready(ssd1306_dev)) {
    LOG_ERR("SSD1306 display device not ready");
    return;
  }
  uint8_t blank_pixels[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  struct display_buffer_descriptor blank_desc = {
    .buf_size = sizeof(blank_pixels),
    .width = 8,
    .height = 8,
    .pitch = 8,
  };
  for (int j = 0; j < 8; j++) {
    for (int i = 0; i < 16; i++) {
      display_write(ssd1306_dev, i * 8, j * 8, &blank_desc, blank_pixels);
    }
  }
  while (true) {
    k_sem_take(&mel_pixbuf_sem, K_FOREVER);
    /* LOG_INF("Drawing to screen"); */
    k_mutex_lock(&mel_pixbuf_mutex, K_FOREVER);
    draw(mel_pixbuf);
    k_mutex_unlock(&mel_pixbuf_mutex);
  }
}

K_THREAD_DEFINE(melvis_draw_thread, 1024,
                melvis_draw_thread_func, NULL, NULL, NULL,
                K_PRIO_PREEMPT(12), 0, 0);
