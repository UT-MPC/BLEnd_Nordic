#ifndef _UTMPCARCSPEAKER_H
#define _UTMPCARCSPEAKER_H

#define SPEAKER_SAMPLE_MAX 8
#include "ble_tss.h"
#include "drv_speaker.h"

static void drv_speaker_evt_handler(drv_speaker_evt_t evt);

static void speaker_control(ble_tss_spkr_mode_t   mode,
                            uint8_t              *p_data,
                            uint16_t              length);

void play_sample_sound(uint8_t id);

void m_speaker_init();

#endif  //_UTMPCARCSPEAKER_H
