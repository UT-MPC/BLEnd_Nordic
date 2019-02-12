#ifndef _UTMPCBLEND_H
#define _UTMPCBLEND_H

#include "app_timer.h"
#include "ble.h"
#include "ble_gap.h"

// Internal event types
#define BLEND_EVT_ADV_REPORT  0x01
#define BLEND_EVT_EPOCH_START  0x02
#define BLEND_EVT_AFTER_SCAN  0x03
// Beacon size in bytes.
#define BEACON_SIZE_B  31
// Reserved length only used in bidirectional mode
#define BLEND_BIDIR_RESERVED_LENGTH  2
// Protocol identifier placed at front
#define BLEND_IDENTIFIER  0xFE
#define BLEND_IDENTIFIER_LENGTH  1
#define BLEND_STACK_RESERVED  4
#define BLEND_USER_PAYLOAD_SIZE  (BEACON_SIZE_B - BLEND_STACK_RESERVED - BLEND_IDENTIFIER_LENGTH)

#define _BLEND_SCAN_RSP  0
#if _BLEND_SCAN_RSP == 0
	#define _BLEND_ADV_TYPE					BLE_GAP_ADV_TYPE_ADV_NONCONN_IND
#else
	#define _BLEND_ADV_TYPE					BLE_GAP_ADV_TYPE_ADV_SCAN_IND
#endif

#define	APP_TIMER_MS(TICKS) TICKS*(1000 * (APP_TIMER_CONFIG_RTC_FREQUENCY + 1))/(uint64_t) APP_TIMER_CLOCK_FREQ

#define ONE_BEACON_MS					5

/**
 * Internal error codes used in Blend.
 */
enum blend_error_t{
  BLEND_NO_ERROR,
  BLEND_DATA_OVERFLOW,
  BLEND_BI_DIR_OVERFLOW,
  BLEND_DATA_IN_SINK_MODE,
};

/**
 * Enum type for neighbor discovery modes.
 */
enum blend_mode_t{
  BLEND_MODE_FULL,
  BLEND_MODE_UNI,
  BLEND_MODE_BI,
  BLEND_MODE_SINK // for debug purpose.
};

/**
 * Data structure for protocol configuration.
 */
typedef struct
{
  uint16_t epoch_length_ms; /*!< Epoch length in ms. */
  uint16_t adv_interval_ms; /*!< Interval between advertisements. */
  enum blend_mode_t blend_mode; /*!< Neighbor discovery mode. */
} blend_param_t;

/**
 * User data packet.
 */
typedef struct
{
  uint8_t* data; /*!< User data inserted in the beacon. */
  uint8_t data_length;
} blend_data_t;

/**
 * Blend internal events.
 */
typedef struct
{
  uint8_t evt_id; /*!< Event id. */
  blend_data_t evt_data; /*!< Event content. */
  ble_gap_addr_t peer_addr; /*!< Peer addr sent by this event. */
} blend_evt_t;

typedef uint32_t blend_ret_t;

typedef void (*blend_evt_handler_t)(blend_evt_t * m_blend_evt);

//! Start discovery process.
/*!
  Function to start the BLEnd neighbor discovery process.
 */
void blend_sched_start(void);

//! Initialize BLEnd module.
/*!
  \param  blend_param protocol parameters.
  \param  handler for handling Blend internal events.
 */
#ifdef BLEND_SDK_14
void blend_init(blend_param_t blend_param, blend_evt_handler_t handler);
#endif

#ifdef BLEND_SDK_THINGY
#include "m_ble.h"
void blend_init(blend_param_t blend_param, blend_evt_handler_t handler, m_ble_service_handle_t* m_ble_service_handles);
#endif

//! Set advertising data.
/*!
  Set advertising content for the beacons.
  \param user_data pointer to the user data payload.
*/
blend_ret_t blend_advdata_set(blend_data_t* user_data);

#endif    // _UTMPCBLEND_H
