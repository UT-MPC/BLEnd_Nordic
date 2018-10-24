#ifndef _BLEND_H
#define _BLEND_H

#include "app_timer.h"
#include "ble.h"
#include "ble_gap.h"


///////////////////////////////////////////

/**
	Blend event type definition
*/
#define BLEND_EVT_ADV_REPORT				0x01
#define BLEND_EVT_EPOCH_START				0x02
#define BLEND_EVT_AFTER_SCAN				0x03


#define BLEND_MAX_Beacon_size 				31									// Max BLE Payload size 26. Total number of a beacon is 31 bytes. 3 for flags field. 2 for identifier. That leaves 26 bytes for payload.
#define BLEND_Bi_Beacon_Reserved 			2									//number of bytes used by blend protocol to work
#define BLEND_IDENTIFIER					0xFE								// Identifier of Blend App at the beginning of the beacon.
#define BLEND_IDENTIFIER_LENGTH				1
#define BLEND_STACK_RESERVED				4
#define _BLEND_SCAN_RSP						0
#define BLEND_MAX_PAYLOAD_SIZE				(BLEND_MAX_Beacon_size - BLEND_STACK_RESERVED - BLEND_IDENTIFIER_LENGTH)		// 26

#if _BLEND_SCAN_RSP == 0
	#define _BLEND_ADV_TYPE					BLE_GAP_ADV_TYPE_ADV_NONCONN_IND
#else
	#define _BLEND_ADV_TYPE					BLE_GAP_ADV_TYPE_ADV_SCAN_IND
#endif

#define	APP_TIMER_MS(TICKS) TICKS*(1000 * (APP_TIMER_CONFIG_RTC_FREQUENCY + 1))/(uint64_t) APP_TIMER_CLOCK_FREQ

//////////////////////////////// BEACON INFO
//#define BLEND_COMPANY_IDENTIFIER		0x1011								// Company Identifier from 0x00 to 0x0680 are already registered, and others are reserved by Bluetooth SIG. Use 0x1011 for temporary identifier.	
#define ONE_BEACON_MS					10

enum blend_error_t{
	BLEND_NO_ERROR,
	BLEND_DATA_OVERFLOW,
	BLEND_Bi_MODE_OVERFLOW
};

enum blend_mode_t{
	BLEND_TYPE_FULL,
	BLEND_TYPE_Uni,
	BLEND_TYPE_Bi
};

/**	This class is used to initialize Blend module. It contains the configurations of Blend

*/
typedef struct
{
	uint16_t epoch_during_in_ms;			//The duration of an epoch. Example: 2000ms
	uint16_t beacon_interval_in_ms;			//The interval between beacons. Example: 77ms
	enum blend_mode_t blend_mode;						
} blend_param_t;

/**	This class is used to transfer data from and to Blend module. 

*/
typedef struct
{
	uint8_t* data;							//The data that will be transimitted by blend.
	uint8_t data_length ;					//The length of the data
} blend_data_t;

/**	This class is used to transfer Blend event data. 

*/
typedef struct
{
  uint8_t 	evt_id;           		/**< Event id. */
  blend_data_t 		evt_data;             /**< Event content.*/
  ble_gap_addr_t peer_addr;					/**< Peer addr sent by this event.*/
} blend_evt_t;

typedef uint32_t blend_ret_t;

typedef void (*blend_evt_handler_t)(blend_evt_t * m_blend_evt);

/** @brief Blend Start
*	This function is used to start the blend module.
*/
void blend_sched_start(void);

/** @brief Blend initialization
*	This function is used to initial the blend module.
*	@param[in]  input_param        		The input init params.
*   @param[in]  handler   				Function to be executed when a blend event happens.
*	
*/
#ifdef BLEND_SDK_14
void blend_init(blend_param_t input_param, blend_evt_handler_t handler);
#endif

#ifdef BLEND_SDK_THINGY
#include "m_ble.h"
void blend_init(blend_param_t input_param, blend_evt_handler_t handler, m_ble_service_handle_t* m_ble_service_handles);
#endif

/** @brief Blend DataSet
*	This function is used to set beacon data for the blend module.
*	@param[in]  input_data        		The pointer to the data you want to send
*/
blend_ret_t blend_advdata_set(blend_data_t* input_data);
#endif
