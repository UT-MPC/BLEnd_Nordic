#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
// new lib

#include "bsp.h"
#include "nrf_soc.h"

//
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_pwr_mgmt.h"
#include "ble_advdata.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "blend.h"

#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */


#define UUID16_SIZE             2                                       /**< Size of 16 bit UUID */
#define UUID32_SIZE             4                                       /**< Size of 32 bit UUID */
#define UUID128_SIZE            16                                      /**< Size of 128 bit UUID */

#define ECHOBACK_BLE_UART_DATA  1                                       /**< Echo the UART data that is received over the Nordic UART Service back to the sender. */


APP_TIMER_DEF (random_timer);

BLE_NUS_C_DEF(m_ble_nus_c);                                             /**< BLE NUS service client instance. */
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< DB discovery module instance. */
//#define m_epoch_during 2000
//#define m_beacon_interval 77
blend_param_t m_blend_param = { 2000, 77, BLEND_MODE_FULL};

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#define APP_DEVICE_NUM                  0x02                              
#define MAX_DEVICE						10
#define m_data_len 						11 
#define discover_index					1
uint8_t payload[m_data_len] = {APP_DEVICE_NUM,
									0x00, 0x00,
									0x00, 0x00,
									0x00, 0x00,
									0xfe, 0xee,
									0xee, 0xef};						/* Discovered Device Count*/
blend_data_t m_blend_data;
									
uint32_t start_tick = 0;
static uint32_t start_tick2 = 0;

static uint32_t reduce_ticks = 0;
static uint32_t epoch_count = 0;
static uint8_t delay_epoches = 3;
uint8_t found_device[MAX_DEVICE];


int get_random(int start,int end){
	
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

/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}



/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    //ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
	
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint16_t index = 0;
    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
            {
                index = 0;
            }
            break;

        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


/**
 * @brief Function for shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    //APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the UART. */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing the Power manager. */
static void power_init(void)
{
    ret_code_t err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the Database Discovery Module. */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}

static void random_timer_handler(void * p_context){
	//starting the blend scheduling.
	memset(&found_device, 0, sizeof(found_device));
	start_tick2 = app_timer_cnt_get();
	blend_sched_start();
}

static void application_timer_set(void)
{
	ret_code_t err_code;
	err_code = app_timer_create(&random_timer, APP_TIMER_MODE_SINGLE_SHOT, random_timer_handler);
	APP_ERROR_CHECK(err_code);
}

///////////////////////////////////////////////////////////////myfunc


static void run_test(){
	ret_code_t err_code;
	
	int random_duration = get_random(0,m_blend_param.epoch_length_ms);

	err_code=app_timer_start(random_timer,APP_TIMER_TICKS(random_duration),NULL);
	APP_ERROR_CHECK(err_code);
	start_tick = app_timer_cnt_get();
}
static void set_blend_data()
{
	m_blend_data.data_length = m_data_len;
	m_blend_data.data = payload;
	if (!blend_advdata_set(&m_blend_data))
	{
		NRF_LOG_ERROR("Blend data set error")
	}
}

static void m_blend_handler(blend_evt_t * p_blend_evt)
{
	if (p_blend_evt->evt_id == BLEND_EVT_ADV_REPORT)
	{
		uint8_t * p_data = p_blend_evt->evt_data.data;
//		uint8_t dlen = p_blend_evt->evt_data.data_length;
//		NRF_LOG_HEXDUMP_INFO(p_data, dlen);
		int now_device=p_data[0];
		uint32_t now_time = app_timer_cnt_get();
		uint8_t dis_flag = 0;
		if (found_device[now_device -1 ] == 1 ) dis_flag = 1;
		if ((dis_flag == 0) && (epoch_count >= delay_epoches)){
			found_device[now_device -1 ] = 1;
			uint32_t time = app_timer_cnt_diff_compute(now_time,start_tick)- reduce_ticks;
			time = _BLEND_APP_TIMER_MS(time) & 0xffff;
			payload [discover_index-2+now_device*2] = time >> 8;
			payload [discover_index-1+now_device*2] = time & 0xff;
			set_blend_data();
		}
		NRF_LOG_INFO("Hear from device #%d \n",now_device);
		return;
	}
	if (p_blend_evt->evt_id == BLEND_EVT_EPOCH_START)
	{
		epoch_count += 1;
		if (epoch_count == delay_epoches ){
			reduce_ticks = app_timer_cnt_diff_compute(app_timer_cnt_get(),start_tick2);
		}
		NRF_LOG_INFO("Epoch #%d started", epoch_count);
		bsp_board_led_invert(BSP_BOARD_LED_0);

	}
	
		
		
}
/////////////////////////////////////////////////////////////////////////
int main(void)
{
	log_init();
    timer_init();
    power_init();
    uart_init();
    buttons_leds_init();
    db_discovery_init();  
	gatt_init();	
    // Start scanning for peripherals and initiate connection
    // with devices that advertise NUS UUID.
    NRF_LOG_INFO("BLE UART central example started.\n");
	application_timer_set();
	// Initialize Blend module with your param and handler
	blend_init(m_blend_param, m_blend_handler);
	bsp_board_led_on(BSP_BOARD_LED_0);
	// Noop loop in order to randomize each device.
	
	//Setting the blend payload. 
	set_blend_data();
	run_test();
    for (;;)
    {
		if (NRF_LOG_PROCESS() == false)
        {
            nrf_pwr_mgmt_run();
        }
    }
}
