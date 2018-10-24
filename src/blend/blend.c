#include "nordic_common.h"
#include "blend.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "ble_advdata.h"
#include "ble.h"
#include "ble_gap.h"

#ifdef BLEND_SDK_15
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_ble_scan.h"
#endif 
#ifdef BLEND_SDK_14
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#endif

#ifdef BLEND_SDK_THINGY
#include "m_ble.h"
#include "ble_uis.h"
static m_ble_service_handle_t*  _blend_service_handles;

#endif

#define APP_BLE_CONN_CFG_TAG    1                                       /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   1                                       /**< Application's BLE observer priority. You shoulnd't need to modify this value. */

#define SCAN_INTERVAL           0x0200                                  /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0200                                  /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT            0x0000                                  /**< Timout when scanning. in second 0x0000 disables timeout. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(_beacon_interval_ms + 100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define scan_timer_mode APP_TIMER_MODE_SINGLE_SHOT 					/*one time count*/
#define epoch_timer_mode APP_TIMER_MODE_REPEATED  					/*repeat*/


#define _blend_reserved_idx (BLEND_MAX_Beacon_size - BLEND_Bi_Beacon_Reserved)
#define _blend_payload_index (BLEND_IDENTIFIER_LENGTH + BLEND_STACK_RESERVED)

#if defined(BLEND_SDK_THINGY) || defined(BLEND_SDK_14)
static ble_gap_scan_params_t const m_scan_params =
{
    .active   = _BLEND_SCAN_RSP,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION <= 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION >= 3)
        .use_whitelist = 0,
    #endif
};
#endif
#if defined(BLEND_SDK_15)
ble_data_t _blend_scan_buffer;
static ble_gap_scan_params_t const m_scan_params =
{
    .active   			= 1,
    .interval 			= SCAN_INTERVAL,
    .window   			= SCAN_WINDOW,
    .timeout  			= SCAN_TIMEOUT,
    .filter_policy 		= BLE_GAP_SCAN_FP_ACCEPT_ALL,
	.scan_phys			= BLE_GAP_PHY_1MBPS,
};

static uint8_t              _blend_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              _blend_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */
static nrf_ble_scan_t _blend_scan_instance;

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t _blend_adv_data =
{
    .adv_data =
    {
        .p_data = _blend_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
};

#endif
// Blend schedule values
// Variables that needs to change in initialization
static uint16_t _epoch_during_in_ms = 2000;
static uint16_t _beacon_interval_ms	= 100;
static uint16_t _scan_during_in_ms = 120;
static int _mid_beacon = 0 ;
uint8_t * _special_beacons;
static uint8_t _blend_mode = BLEND_TYPE_FULL;
static blend_evt_handler_t _blend_evt_handler;
// Variables that does NOT need to change in initialization
static uint32_t _blend_epoch_start = 0;
static uint8_t _blend_on_beacon_flag = 0;
static uint8_t _blend_sent_beacon_count = 0;
static ble_gap_adv_params_t _blend_adv_params;                                 /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t _epoch_flag = 0;

uint8_t _blend_beacon_info[31];

APP_TIMER_DEF (scan_timer);
APP_TIMER_DEF (half_epoch_timer);
APP_TIMER_DEF (beacon_count_timer);
APP_TIMER_DEF (one_beacon_timer);
APP_TIMER_DEF (beacon_slack_timer);


static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

int _blend_get_random(int start,int end){
	
	uint8_t random_number[4];
	ret_code_t err_code;
	uint8_t pool_avail = 0;
	
	//if there is a randon number available
	while (pool_avail <4)
    {
		err_code = sd_rand_application_bytes_available_get(&pool_avail);
		APP_ERROR_CHECK(err_code);       
    }
	err_code = sd_rand_application_vector_get(random_number, 4);
    APP_ERROR_CHECK(err_code);
	int dice = abs(random_number[0]+(random_number[1]<<8)+(random_number[2]<<16) + (random_number[3]<<24));
	return (int)(dice % (end-start))+start;
}


#if defined(BLEND_SDK_14) || defined(BLEND_SDK_THINGY)
void advertising_set(void)
{
    uint32_t      err_code;
	/* set the flags */
	_blend_beacon_info[0] = 0x02;
	_blend_beacon_info[1] = 0x01;
	_blend_beacon_info[2] = 0x04;
	
	/* set the blend length */
	_blend_beacon_info[3] = BLEND_MAX_Beacon_size - 4;
	/* set the identifier for blend */
	_blend_beacon_info[4] = BLEND_IDENTIFIER;
	#if _BLEND_SCAN_RSP == 0
		err_code = sd_ble_gap_adv_data_set(_blend_beacon_info, BLEND_MAX_Beacon_size, NULL, 0);
	#else
		err_code = sd_ble_gap_adv_data_set(_blend_beacon_info, BLEND_MAX_Beacon_size, _blend_beacon_info + 3, 28);
	#endif
	
    APP_ERROR_CHECK(err_code);
}

void advertising_init(void)
{
    // Initialize advertising parameters (used when starting advertising).
    memset(&_blend_adv_params, 0, sizeof(_blend_adv_params));
    _blend_adv_params.type        = _BLEND_ADV_TYPE;
    _blend_adv_params.p_peer_addr = NULL;    // Undirected advertisement.
    _blend_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    _blend_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    _blend_adv_params.timeout     = 0;       // Never time out.
	advertising_set();
}
 void blend_parse(ble_gap_evt_adv_report_t const * p_adv_report)
{

    uint16_t     index  = 0;
	uint16_t countdown;
    uint8_t    * p_data = (uint8_t *)p_adv_report->data;
	
	
	while (index < p_adv_report->dlen)
    {
		uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == BLEND_IDENTIFIER)
        {
//			if (p_adv_report->scan_rsp == 1){
//				NRF_LOG_DEBUG("This is a scan response data !!!!!!!!!!!!!!!!!!!!");
//			}
//			NRF_LOG_HEXDUMP_DEBUG(p_data, p_adv_report->dlen);
//			NRF_LOG_HEXDUMP_DEBUG(p_adv_report->peer_addr.addr, 6);
//			NRF_LOG_DEBUG("get message from addr type: %d", p_adv_report->peer_addr.addr_type);
			uint32_t now_time = app_timer_cnt_get();
			if (_blend_mode == BLEND_TYPE_Bi){
				countdown = (p_data[_blend_reserved_idx] << 8) + p_data[_blend_reserved_idx+1];
				if (countdown == 0xffff) countdown =0;
				countdown = countdown - _scan_during_in_ms + APP_TIMER_MS(app_timer_cnt_diff_compute(now_time, _blend_epoch_start));
				_special_beacons[countdown / _beacon_interval_ms +1] = 1;
			}
			blend_evt_t new_blend_evt;
			new_blend_evt.evt_id = BLEND_EVT_ADV_REPORT;
			new_blend_evt.evt_data.data = p_data + _blend_payload_index;
			new_blend_evt.evt_data.data_length = field_length - 1;
			new_blend_evt.peer_addr = p_adv_report->peer_addr;
			(*_blend_evt_handler) ( &new_blend_evt);
        }
        index += field_length + 1;
    }
}
#endif

#if defined(BLEND_SDK_15)

void advertising_set(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = BLEND_IDENTIFIER;

    manuf_specific_data.data.p_data = (uint8_t *) _blend_beacon_info;
    manuf_specific_data.data.size   = BLEND_MAX_Beacon_size;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    err_code = ble_advdata_encode(&advdata, _blend_adv_data.adv_data.p_data, &_blend_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&_blend_adv_handle, &_blend_adv_data, &_blend_adv_params);
    APP_ERROR_CHECK(err_code);
}

void advertising_init(void)
{
    // Initialize advertising parameters (used when starting advertising).
    memset(&_blend_adv_params, 0, sizeof(_blend_adv_params));

    _blend_adv_params.properties.type			= BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    _blend_adv_params.p_peer_addr 				= NULL;    // Undirected advertisement.
    _blend_adv_params.filter_policy         	= BLE_GAP_ADV_FP_ANY;
    _blend_adv_params.interval    				= NON_CONNECTABLE_ADV_INTERVAL;
    _blend_adv_params.duration     				= 0;       // Never time out.
	advertising_set();
}
void blend_parse(ble_gap_evt_adv_report_t const * p_adv_report)
{
	uint16_t     index  = 0;
	uint16_t countdown;
    uint8_t    * p_data = (uint8_t *)p_adv_report->data.p_data;
	//NRF_LOG_INFO("get message from addr type: %d", p_adv_report->peer_addr.addr_type)
	 while (index < p_adv_report->data.len)
    {
       uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA)
        {
			if ((p_data[index+2]== (BLEND_IDENTIFIER & 0x00ff) )&&(p_data[index+3]==(BLEND_IDENTIFIER >> 8)))
			{
				uint32_t now_time = app_timer_cnt_get();
				countdown = (p_data[index + 4 + _blend_reserved_idx] << 8) + p_data[index + 4 + _blend_reserved_idx+1];
				if (countdown == 0xffff) countdown =0;
				countdown = countdown - _scan_during_in_ms + APP_TIMER_MS(app_timer_cnt_diff_compute(now_time, _blend_epoch_start));
				_special_beacons[countdown / _beacon_interval_ms +1] = 1;
				
				blend_evt_t new_blend_evt;
				new_blend_evt.evt_id = BLEND_EVT_ADV_REPORT;
				new_blend_evt.evt_data.data = p_data + index + 4 + _blend_payload_index;
				new_blend_evt.evt_data.data_length = field_length - BLEND_Reserved_size - 3;
				(*_blend_evt_handler) ( &new_blend_evt);
			}
        }
        index += field_length + 1;
    }
	ret_code_t ret = sd_ble_gap_scan_start(NULL, &_blend_scan_instance.scan_buffer);
	APP_ERROR_CHECK(ret);
}

#endif
/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            ble_gap_evt_adv_report_t const * p_adv_report = &p_gap_evt->params.adv_report;
			blend_parse(p_adv_report);
        }break; // BLE_GAP_EVT_ADV_REPORT

        default:
            break;
    }
}

#ifdef BLEND_SDK_THINGY
/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void blend_ble_stack_init(void )
{
	m_ble_init_t             ble_params;
	ret_code_t err_code;
	/**@brief Initialize BLE handling module. */
    ble_params.evt_handler       = ble_evt_handler;
    ble_params.p_service_handles = _blend_service_handles;
    ble_params.service_num       = THINGY_SERVICES_MAX;

    err_code = m_ble_init(&ble_params);
    APP_ERROR_CHECK(err_code);
}
#endif

#if defined(BLEND_SDK_14)
static void blend_ble_stack_init(void )
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(_blend_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}
#endif


#ifdef BLEND_SDK_15
static void blend_ble_stack_init(void )
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(_blend_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
	_blend_scan_instance.scan_buffer.p_data = _blend_scan_instance.scan_buffer_data;
	_blend_scan_instance.scan_buffer.len = NRF_BLE_SCAN_BUFFER;
}

#endif 
#ifdef BLEND_SDK_15
void _blend_beacon_sd_start(){
	ret_code_t err_code;
	err_code = sd_ble_gap_adv_start(_blend_adv_handle, APP_BLE_CONN_CFG_TAG);
	APP_ERROR_CHECK(err_code);
}
void _blend_beacon_sd_stop(){
	ret_code_t err_code;
	err_code = sd_ble_gap_adv_stop(_blend_adv_handle);
	APP_ERROR_CHECK(err_code);
}
void _blend_scan_sd_start(){
	ret_code_t err_code;
	err_code = sd_ble_gap_scan_start(&m_scan_params, &_blend_scan_instance.scan_buffer);
	APP_ERROR_CHECK(err_code);
}

#endif 
#if defined(BLEND_SDK_14) || defined(BLEND_SDK_THINGY)
void _blend_beacon_sd_start(){
	ret_code_t err_code;
    err_code = sd_ble_gap_adv_start(&_blend_adv_params, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
}
void _blend_beacon_sd_stop(){
	ret_code_t err_code;
	err_code = sd_ble_gap_adv_stop();
	APP_ERROR_CHECK(err_code);
}
void _blend_scan_sd_start(){
	ret_code_t err_code;
	err_code = sd_ble_gap_scan_start(&m_scan_params);
	APP_ERROR_CHECK(err_code);
}
#endif

void beacon_slack_timer_handler(void){
	_blend_beacon_sd_start();
	_blend_on_beacon_flag = 1;

}
void advertising_start(void)
{
    ret_code_t err_code;
	uint32_t slack_during = _blend_get_random(0,10);
	if (_blend_on_beacon_flag == 0){
		if (slack_during == 0){
			beacon_slack_timer_handler();
		}
		else{
			err_code =app_timer_start(beacon_slack_timer,APP_TIMER_TICKS(slack_during),NULL);
			APP_ERROR_CHECK(err_code);
		}	
	}
	_blend_on_beacon_flag = 1;
}
void advertising_stop(void)
{
	if (_blend_on_beacon_flag ==1){
		_blend_beacon_sd_stop();
		_blend_on_beacon_flag = 0;
	}
}

void beacon_count_set(int num)
{
	uint16_t now_countdown;
	now_countdown = _epoch_during_in_ms - APP_TIMER_MS(app_timer_cnt_diff_compute(app_timer_cnt_get(),_blend_epoch_start));
	if (num == -1){
		now_countdown = ONE_BEACON_MS;
	}
	if (_blend_mode == BLEND_TYPE_Bi){
		_blend_beacon_info[_blend_reserved_idx] = (now_countdown & 0xff00) >> 8;
		_blend_beacon_info[_blend_reserved_idx+1] = now_countdown & 0xff;
	}
	advertising_set();
}


void one_beacon_timer_handler()
{
	ret_code_t ret;
	advertising_stop();
	if (_epoch_flag == 1){
		_blend_epoch_start = app_timer_cnt_get();	
		_blend_scan_sd_start();
		NRF_LOG_DEBUG("In Blend module: scan start");
		ret =app_timer_start(scan_timer,APP_TIMER_TICKS(_scan_during_in_ms),NULL);
		APP_ERROR_CHECK(ret);
	}
}
/**@brief Function to start scanning. */
void scan_prepare(void)
{
    ret_code_t ret;
	beacon_count_set(-1);

	beacon_slack_timer_handler();
	ret = app_timer_start(one_beacon_timer,APP_TIMER_TICKS(ONE_BEACON_MS),NULL);
	APP_ERROR_CHECK(ret);
	//scan_tick = app_timer_cnt_get();

}

void scan_stop(void)
{
    ret_code_t ret;

    ret = sd_ble_gap_scan_stop();
    APP_ERROR_CHECK(ret);
	NRF_LOG_DEBUG("In Blend module: scan stopped");
	//NRF_LOG_HEXDUMP_DEBUG(_blend_scan_instance.scan_buffer.p_data, _blend_scan_instance.scan_buffer.len);

}


void scan_timer_handler(void* p_context)
{
	ret_code_t ret;
	scan_stop();
	ret=app_timer_stop(scan_timer);
    APP_ERROR_CHECK(ret);
	ret_code_t err_code =app_timer_start(beacon_count_timer,APP_TIMER_TICKS (_beacon_interval_ms),NULL);
	APP_ERROR_CHECK(err_code);
	_blend_sent_beacon_count = 0;
	beacon_count_set (_blend_sent_beacon_count);
	advertising_start();
	
	//Call blend handler
	blend_evt_t new_blend_evt;
	new_blend_evt.evt_id = BLEND_EVT_AFTER_SCAN;
	new_blend_evt.evt_data.data = NULL;
	new_blend_evt.evt_data.data_length = 0;
	(*_blend_evt_handler) ( &new_blend_evt);
	//NRF_LOG_INFO("scan stopped")

}

void half_epoch_timer_handler (void * p_context)
{
	ret_code_t err_code;
	if (_epoch_flag == 0)
	{
		_epoch_flag = 1;
		advertising_stop();
		memset(_special_beacons, 0, (ROUNDED_DIV(_epoch_during_in_ms , _beacon_interval_ms) + 2) * sizeof(_special_beacons[0]));
		//Call blend handler
		blend_evt_t new_blend_evt;
		new_blend_evt.evt_id = BLEND_EVT_EPOCH_START;
		new_blend_evt.evt_data.data = NULL;
		new_blend_evt.evt_data.data_length = 0;
		(*_blend_evt_handler) ( &new_blend_evt);
		
		scan_prepare();
		err_code=app_timer_stop(beacon_count_timer);
		APP_ERROR_CHECK(err_code);
	}
	else
	{
		_epoch_flag = 0;
	}
}

void beacon_count_timer_handler (void * p_context)
{
	advertising_stop();
	_blend_sent_beacon_count += 1;
	beacon_count_set (_blend_sent_beacon_count);
	
	if ((_blend_sent_beacon_count <= _mid_beacon) || (_blend_mode == 1)){
		advertising_start();	
	}

	if ((_blend_sent_beacon_count > _mid_beacon) && (_special_beacons[_blend_sent_beacon_count] == 1) && (_blend_mode == 3))
	{
		advertising_start();
	}
}
void blend_sched_start()
{
	ret_code_t err_code;
	_epoch_flag = 1;
	err_code=app_timer_start(half_epoch_timer,APP_TIMER_TICKS(_epoch_during_in_ms /2),NULL);
	APP_ERROR_CHECK(err_code);
	scan_prepare();
}



void blend_timer_set(void)
{
	// The timer for scanning duration.
	ret_code_t err_code = app_timer_create(&scan_timer, scan_timer_mode, scan_timer_handler);
	APP_ERROR_CHECK(err_code);
	
	// The timer for half epoch duration in order to use BiBlend.
	err_code = app_timer_create(&half_epoch_timer, epoch_timer_mode, half_epoch_timer_handler);
	APP_ERROR_CHECK(err_code);
	
	// The timer for one beacon. This timer times out after every beacon sent in order to change the content of the beacons.
	err_code = app_timer_create(&beacon_count_timer, APP_TIMER_MODE_REPEATED, beacon_count_timer_handler);
	APP_ERROR_CHECK(err_code);
	
	// The timer for scheduling one beacon before and after scanning.
	err_code = app_timer_create(&one_beacon_timer, scan_timer_mode, one_beacon_timer_handler);
	APP_ERROR_CHECK(err_code);
	
	// The timer for beacon slack. 0-10ms.
	err_code = app_timer_create(&beacon_slack_timer, APP_TIMER_MODE_SINGLE_SHOT, beacon_slack_timer_handler);
	APP_ERROR_CHECK(err_code);
}

void blend_param_set(blend_param_t input)
{
	_epoch_during_in_ms = input.epoch_during_in_ms;
	_beacon_interval_ms = input.beacon_interval_in_ms;
    _scan_during_in_ms = (_beacon_interval_ms + 5 + 10);
	_special_beacons = (uint8_t *) malloc( (ROUNDED_DIV(_epoch_during_in_ms , _beacon_interval_ms) + 2) * sizeof(uint8_t)) ;
	_mid_beacon = ((_epoch_during_in_ms / 2) - _scan_during_in_ms + _beacon_interval_ms / 2 ) / _beacon_interval_ms ;
	_blend_mode = input.blend_mode;
}

blend_ret_t blend_advdata_set(blend_data_t *input)
{
	uint8_t dlen = input->data_length;
	uint8_t * payload = input->data;
	
	if (dlen > BLEND_MAX_PAYLOAD_SIZE){
		return BLEND_DATA_OVERFLOW;
	}
	
	if (_blend_mode == BLEND_TYPE_Bi) {
		if (dlen > BLEND_MAX_PAYLOAD_SIZE - BLEND_Bi_Beacon_Reserved){
			return BLEND_Bi_MODE_OVERFLOW;
		}
	}
	for (int i = 0; i < dlen; i++)
	{
		_blend_beacon_info[_blend_payload_index + i] = payload[i];
	}
	advertising_set();
	return BLEND_NO_ERROR;
}

#ifdef BLEND_SDK_15
void blend_init(blend_param_t input, blend_evt_handler_t handler)
{
	NRF_LOG_DEBUG("Blend init start\n");
	blend_param_set(input);
	timer_init();
	blend_ble_stack_init();
	advertising_init();
	blend_timer_set();
	_blend_evt_handler = handler;
	memset(_blend_beacon_info, 0, sizeof(_blend_beacon_info));
	NRF_LOG_DEBUG("Blend init finished\n");

}
#endif

#ifdef BLEND_SDK_14
void blend_init(blend_param_t input, blend_evt_handler_t handler)
{
	blend_param_set(input);
	timer_init();
	blend_ble_stack_init();
	advertising_init();
	blend_timer_set();
	_blend_evt_handler = handler;
	memset(_blend_beacon_info, 0, sizeof(_blend_beacon_info));
}
#endif

#ifdef BLEND_SDK_THINGY
void blend_init(blend_param_t input, blend_evt_handler_t handler, m_ble_service_handle_t* m_ble_services_handler)
{
	_blend_service_handles = m_ble_services_handler;
	blend_param_set(input);
	timer_init();
	blend_ble_stack_init();
	advertising_init();
	blend_timer_set();
	_blend_evt_handler = handler;
	memset(_blend_beacon_info, 0, sizeof(_blend_beacon_info));
}
#endif