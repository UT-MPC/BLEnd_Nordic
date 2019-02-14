#include "drv_mic.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "nrf_log.h"

#if (CONFIG_AUDIO_CODEC == CONFIG_AUDIO_CODEC_SOUND_LEVEL)

void float2Bytes(uint8_t* bytes_temp[4],float float_variable){ 
  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = float_variable;
  memcpy(bytes_temp, thing.bytes, 4);
}

void drv_audio_coder_init(void) {
  NRF_LOG_DEBUG("Audio Codec selected: SOUND LEVEL");
}

void drv_audio_coder_encode(int16_t *raw_samples, m_audio_frame_t *p_frame) {
  int num_samples = (CONFIG_AUDIO_FRAME_SIZE_SAMPLES * sizeof(int16_t)) / 2;
  int64_t abs_sum = 0;
  int16_t *sample_iter = raw_samples;
  for (int i = 0; i < num_samples; ++i) {
    abs_sum += (*sample_iter)*(*sample_iter);
    sample_iter++;
  }
  float avg_amp = sqrt(abs_sum / (float) (num_samples));
  float res = 20*log10(avg_amp / 32767);

  //NRF_LOG_DEBUG("noise_level = " NRF_LOG_FLOAT_MARKER "\r\n: ", NRF_LOG_FLOAT(res));
  
  float2Bytes((uint8_t*) p_frame->data, res); // Write the result to the first 4 bytes of output stream.
  p_frame->data_size = num_samples; // to represent a float
}

#endif // (CONFIG_AUDIO_CODEC == CONFIG_AUDIO_CODEC_ADPCM)
