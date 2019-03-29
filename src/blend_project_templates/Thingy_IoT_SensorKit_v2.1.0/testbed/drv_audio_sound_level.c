#include "drv_mic.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "nrf_log.h"
#include "context.h"

#define MICROPHONE_ADPCM_RAW_THRESHOLD 800

#if (CONFIG_AUDIO_CODEC == CONFIG_AUDIO_CODEC_SOUND_LEVEL)

void float2Bytes(uint8_t* bytes_temp,float float_variable){ 
  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = float_variable;
  memcpy(bytes_temp, thing.bytes, 4);
}

void int2Bytes(uint8_t* bytes_temp,int int_variable){ 
  union {
    int a;
    unsigned char bytes[4];
  } thing;
  thing.a = int_variable;
  memcpy(bytes_temp, thing.bytes, 4);
}

int16_t getPositive(int16_t val) {
  if (val < 0) {
    return val*(-1);
  } else {
    return val;
  }
}

void drv_audio_coder_init(void) {
  NRF_LOG_DEBUG("Audio Codec selected: SOUND LEVEL");
}

void drv_audio_coder_encode(int16_t *raw_samples, m_audio_frame_t *p_frame) {
  int num_samples = (CONFIG_AUDIO_FRAME_SIZE_SAMPLES * sizeof(int16_t)) / 2;
  int64_t abs_sum = 0;
  int16_t *sample_iter = raw_samples;
  
  int peak_in_frame = 0;
  int count_over_thres = 0;
  int sum_all_samples = 0;
  int sum_over_thres_samples = 0;

  //int16_t samples_over_thres[CONFIG_AUDIO_FRAME_SIZE_SAMPLES];
  //int16_t samples[CONFIG_AUDIO_FRAME_SIZE_SAMPLES];

  for (int i = 0; i < num_samples; ++i) {
    int16_t val = getPositive((*sample_iter));
    //samples[i] = val;
    if (val > peak_in_frame) {
      peak_in_frame = val;
    }

    if (val > MICROPHONE_ADPCM_RAW_THRESHOLD) {
      //samples_over_thres[count_over_thres] = val;
      count_over_thres++;
      sum_over_thres_samples += val;
    }
    sum_all_samples += val;

    abs_sum += (*sample_iter)*(*sample_iter);
    sample_iter++;
  }

  float avg_all_samples = ((float) sum_all_samples) / ((float) num_samples);
  float avg_over_thres_samples =  ((float) sum_over_thres_samples) / ((float) count_over_thres);

  // int diff;
  // int sum_all_squares = 0;
  // for (int i = 0; i < num_samples; ++i) {
  //   int16_t diff = samples[i] - ((int16_t) avg_all_samples);
  //   int square = ((int)diff)*((int)diff);
  //   sum_all_squares += square;
  // }
  // float stddev_all_samples = sqrt(((float) sum_all_squares) / ((float) num_samples));

  // int sum_over_thres_squares = 0;
  // for (int i = 0; i < count_over_thres; ++i) {
  //   int16_t diff = samples_over_thres[i] - ((int16_t) avg_over_thres_samples);
  //   int square = ((int)diff)*((int)diff);
  //   sum_over_thres_squares += square;
  // }
  // float stddev_over_thres_samples =  sqrt(((float) sum_over_thres_squares) / ((float) count_over_thres));

  // float avg_amp = sqrt(abs_sum / (float) (num_samples));
  // float res = 20*log10(avg_amp / 32767);

  //NRF_LOG_DEBUG("noise_level = " NRF_LOG_FLOAT_MARKER "\r\n: ", NRF_LOG_FLOAT(res));
  
  //NRF_LOG_DEBUG("num_samples = %d \r\n: ", num_samples);
  //NRF_LOG_DEBUG("peak_in_frame = %d \r\n: ", peak_in_frame);
  //NRF_LOG_DEBUG("count_over_thres = %d \r\n: ", count_over_thres);
  //NRF_LOG_DEBUG("avg_all_samples = " NRF_LOG_FLOAT_MARKER "\r\n: ", NRF_LOG_FLOAT(avg_all_samples));
  //NRF_LOG_DEBUG("avg_over_thres_samples = " NRF_LOG_FLOAT_MARKER "\r\n: ", NRF_LOG_FLOAT(avg_over_thres_samples));
  //NRF_LOG_DEBUG("stddev_all_samples = " NRF_LOG_FLOAT_MARKER "\r\n: ", NRF_LOG_FLOAT(stddev_all_samples));
  //NRF_LOG_DEBUG("stddev_over_thres_samples = " NRF_LOG_FLOAT_MARKER "\r\n: ",
  //  NRF_LOG_FLOAT(stddev_over_thres_samples));

  //float2Bytes((uint8_t*) p_frame->data, res); // Write the result to the first 4 bytes of output stream.
  int2Bytes(((uint8_t*) p_frame->data), peak_in_frame);
  int2Bytes(((uint8_t*) p_frame->data) + 4, count_over_thres);
  float2Bytes((uint8_t*) p_frame->data + 8, avg_all_samples);
  float2Bytes((uint8_t*) p_frame->data + 12, avg_over_thres_samples);
  //int2Bytes(((uint8_t*) p_frame->data) + 20, num_samples);

  p_frame->data_size = num_samples; // to represent a float
}

#endif // (CONFIG_AUDIO_CODEC == CONFIG_AUDIO_CODEC_ADPCM)
