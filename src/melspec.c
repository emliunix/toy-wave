#include <string.h>
#include <math.h>
#include <dsp/transform_functions.h>
#include <dsp/complex_math_functions.h>
#include <dsp/basic_math_functions.h>
#include <dsp/fast_math_functions.h>

#include <melspec.h>

#define PASTER(prefix, size, suffix) prefix##size##suffix
#define EVALUATOR(prefix, size, suffix) PASTER(prefix, size, suffix)
#define MELSPEC_FFT_INIT_FN EVALUATOR(arm_rfft_fast_init_, MELSPEC_FFT_SIZE, _f32)

// --- Helper Functions for Mel Scale Conversion ---
static float32_t hz_to_mel(float32_t hz) {
    return 2595.0f * log10f(1.0f + hz / 700.0f);
}

static float32_t mel_to_hz(float32_t mel) {
    return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

/**
 * @brief Generates the Mel filterbank weights. Call this once at initialization.
 */
static void create_mel_filterbank(size_t num_mels, size_t fft_size, int sample_rate,
                                  size_t* filter_indices, float32_t *filter_values
                                  ) {
  float32_t min_mel = hz_to_mel(0);
  float32_t max_mel = hz_to_mel(sample_rate / 2);
  float32_t mel_step = (max_mel - min_mel) / (num_mels + 1);

  // the triangle vertex bins
  size_t right_fft_bin = 0;
  size_t center_fft_bin = 0;
  size_t left_fft_bin = 0;
  size_t val_i = 0;

  float32_t mel_point;
  float32_t hz_point = 0.0f;
  for (int i = 0; i < num_mels + 2; i++) {
    mel_point = min_mel + i * mel_step;
    hz_point = mel_to_hz(mel_point);
    right_fft_bin = (size_t)floorf((fft_size + 1) * hz_point / sample_rate);
    // Create the triangular filters
    if (i >= 2) {
      filter_indices[(i-2)*3] = left_fft_bin;
      filter_indices[(i-2)*3+1] = right_fft_bin;
      filter_indices[(i-2)*3+2] = val_i;
      for (size_t j = left_fft_bin; j < center_fft_bin; j++) {
        filter_values[val_i++] = (float32_t)(j - left_fft_bin) / (center_fft_bin - left_fft_bin);
      }
      for (size_t j = center_fft_bin; j < right_fft_bin; j++) {
        filter_values[val_i++] = (float32_t)(right_fft_bin - j) / (right_fft_bin - center_fft_bin);
      }
    }
    left_fft_bin = center_fft_bin;
    center_fft_bin = right_fft_bin;
  }
}

int melspec_init(melspec_t *mel, float32_t gain, int sample_rate, float32_t *buf, size_t buf_size) {
  int res = 0;
  if (buf_size < MELSPEC_FFT_SIZE) {
    return -1;
  }
  mel->gain = gain;
  mel->buffer = buf;
  mel->capacity = buf_size;
  mel->size = 0;
  arm_rfft_fast_init_f32(&mel->rfft_inst, MELSPEC_FFT_SIZE);
  res = MELSPEC_FFT_INIT_FN(&mel->rfft_inst);
  if (res) {
    return res;
  }
  create_mel_filterbank(MELSPEC_MEL_SIZE, MELSPEC_FFT_SIZE, sample_rate,
                        mel->mel_filterbank_indices, mel->mel_filterbank_values);
  return res;
}

void melspec_put(melspec_t *mel, int16_t *data, size_t data_size) {
  if (mel->size + data_size > mel->capacity) {
    // some drop is required
    if (data_size >= mel->capacity) {
      // shrink input to the right
      data += data_size - mel->capacity;
      data_size = mel->capacity;
      // clear buffer
      mel->size = 0;
    } else {
      // need to shift left
      size_t shift = mel->size - (mel->capacity - data_size);
      melspec_shift(mel, shift);
    }
  }
  // copy data
  float32_t *p = mel->buffer + mel->size;
  for (size_t i = 0; i < data_size; i++) {
    p[i] = (float32_t)data[i] * mel->gain / (float32_t)(0x4000);
  }
  mel->size += data_size;
}

int melspec_get_size(melspec_t *mel) {
  return mel->size;
}

void melspec_shift(melspec_t *mel, size_t stride) {
  stride = stride < mel->size ? stride : mel->size;
  if (stride < mel->size) {
    memmove(mel->buffer, mel->buffer + stride, sizeof(float32_t) * (mel->size - stride));
  }
  mel->size = mel->size - stride;
}

void melspec_process(melspec_t *mel) {
  // 1. fft
  arm_rfft_fast_f32(&mel->rfft_inst, mel->buffer, mel->fft_result, 0);
  // 2. power spectra
  arm_cmplx_mag_f32(mel->fft_result, mel->power_spectrum, MELSPEC_FFT_SIZE / 2);
  arm_mult_f32(mel->power_spectrum, mel->power_spectrum, mel->power_spectrum, MELSPEC_FFT_SIZE / 2);
  // 3. apply mel filterbank
  for (size_t i = 0; i < MELSPEC_MEL_SIZE; i++) {
    size_t s = mel->mel_filterbank_indices[i*3];
    size_t e = mel->mel_filterbank_indices[i*3+1];
    size_t b = mel->mel_filterbank_indices[i*3+2];
    arm_dot_prod_f32(&mel->mel_filterbank_values[b], &mel->power_spectrum[s], e - s, &mel->mel_spectrum[i]);
  }
  // 4. to decibel
  arm_vlog_f32(mel->mel_spectrum, mel->mel_spectrum, MELSPEC_MEL_SIZE);
  arm_scale_f32(mel->mel_spectrum, 10.0f / logf(10.0f), mel->mel_spectrum, MELSPEC_MEL_SIZE);
}

size_t melspec_get_mel_spectrum_size(melspec_t *mel) {
  return MELSPEC_MEL_SIZE;
}

float32_t* melspec_get_mel_spectrum(melspec_t *mel) {
  return mel->mel_spectrum;
}
