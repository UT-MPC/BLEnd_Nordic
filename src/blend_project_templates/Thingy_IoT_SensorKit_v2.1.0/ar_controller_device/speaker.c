#include <string.h>
#include "app_util_platform.h"
#include "ble_tss.h"

#include "drv_speaker.h"
#define  NRF_LOG_MODULE_NAME "arc_speaker       "
#include "nrf_log.h"


static void drv_speaker_evt_handler(drv_speaker_evt_t evt)
{
    switch(evt)
    {
        case DRV_SPEAKER_EVT_FINISHED:
        {
            NRF_LOG_DEBUG("drv_speaker_evt_handler: DRV_SPEAKER_EVT_FINISHED\r\n");
        }
        break;
        //
        case DRV_SPEAKER_EVT_BUFFER_WARNING:
        {
            NRF_LOG_WARNING("drv_speaker_evt_handler: DRV_SPEAKER_EVT_BUFFER_WARNING\r\n");
        }
        break;
        //
        case DRV_SPEAKER_EVT_BUFFER_READY:
        {
            NRF_LOG_DEBUG("drv_speaker_evt_handler: DRV_SPEAKER_EVT_BUFFER_READY\r\n");
        }
        break;
        //
        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
}


static void speaker_control(ble_tss_spkr_mode_t   mode,
                            uint8_t              *p_data,
                            uint16_t              length)
{
    uint32_t err_code;
    if (mode == BLE_TSS_SPKR_MODE_PCM)
    {
        err_code = drv_speaker_ble_pcm_play(p_data, length);
        APP_ERROR_CHECK(err_code);
    }
    else if (mode == BLE_TSS_SPKR_MODE_FREQ)
    {
        ble_tss_spkr_freq_t * p_spkr_freq = (ble_tss_spkr_freq_t *)p_data;

        if (length == sizeof(ble_tss_spkr_freq_t))
        {
            APP_ERROR_CHECK(drv_speaker_tone_start(p_spkr_freq->freq, p_spkr_freq->duration_ms, p_spkr_freq->volume));
        }
    }
    else if (mode == BLE_TSS_SPKR_MODE_SAMPLE)
    {
        ble_tss_spkr_t * p_spkr = (ble_tss_spkr_t *)p_data;

        if (length == sizeof(ble_tss_spkr_sample_t))
        {
            err_code = drv_speaker_sample_play(p_spkr->sample_id);
            APP_ERROR_CHECK(err_code);
        }
    }
}

void play_sample_sound(uint8_t id) {
    ble_tss_spkr_mode_t mode = BLE_TSS_SPKR_MODE_SAMPLE;
    uint16_t length = sizeof(ble_tss_spkr_sample_t);
    uint8_t *p_data = {id};
    speaker_control(mode, p_data, length);
}


void m_speaker_init()
{
    uint32_t           err_code;
    drv_speaker_init_t speaker_init;

    NRF_LOG_INFO("Speaker_init \r\n");

    speaker_init.evt_handler = drv_speaker_evt_handler;
    err_code = drv_speaker_init(&speaker_init);
    APP_ERROR_CHECK(err_code);
}