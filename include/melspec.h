#ifndef _MELSPEC_H
#define _MELSPEC_H

#include <stdint.h>
#include <dsp/transform_functions.h>

#define MELSPEC_MEL_SIZE 32
#define MELSPEC_FFT_SIZE 1024

typedef struct melspec {
  float32_t gain;
  float32_t *buffer;
  size_t capacity;
  size_t size;
  arm_rfft_fast_instance_f32 rfft_inst;
  float32_t fft_result[MELSPEC_FFT_SIZE];
  float32_t power_spectrum[MELSPEC_FFT_SIZE/2];
  size_t mel_filterbank_indices[MELSPEC_MEL_SIZE * 3];
  float32_t mel_filterbank_values[MELSPEC_FFT_SIZE * 2];
  float32_t mel_spectrum[MELSPEC_MEL_SIZE];
} melspec_t;

int melspec_init(melspec_t *mel, float32_t gain, int sample_rate, float32_t *buf, size_t buf_size);
void melspec_put(melspec_t *mel, int16_t *data, size_t data_size);
int melspec_get_size(melspec_t *mel);
void melspec_shift(melspec_t *mel, size_t stride);
size_t melspec_get_mel_size(melspec_t *mel);
void melspec_process(melspec_t *mel);
size_t melspec_get_mel_spectrum_size(melspec_t *mel);
float32_t* melspec_get_mel_spectrum(melspec_t *mel);

#endif /* _MELSPEC_H */
