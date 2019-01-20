
#define _TB_MAIN_CODE_ 
#include "tb_global.h"
#include "eddystone.h"
#include "ibeacon.h"
#include "tbm_packet.h"
#include "tb_manager.h"
#include "ble_bms.h"
#include "advertising.h"
#include "tgsec_ibeacon.h"
#include "timeslot.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT   1                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                       /** When having DFU Service support in application the Service Changed Characteristic should always be present. */

#define DFU_DEVICE_NAME                   "TrgDfu"                                    /**< Name of device. Will be included in the advertising data. */
#define DFU_MANUFACTURER_NAME             "Tangerine Inc."                           　/**< Manufacturer. Will be passed to Device Information Service. */
#define DFU_APP_ADV_INTERVAL              300                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define DFU_APP_ADV_DURATION              18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_BLE_OBSERVER_PRIO             3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_SOC_OBSERVER_PRIO             1                                           /**< Applications' SoC observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG              1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define TS_ADV_INTERVAL                   7625                                        /**< The advertising interval for time-slot based advertisement in ms */
#define BLE_ADV_INTERVAL                  100                                         /**< The advertising interval for BLE-based non-connectable advertisement in ms */
#define BASE_ADV_INTERVAL                 MSEC_TO_UNITS(BLE_ADV_INTERVAL, UNIT_0_625_MS) /**< This value can vary between 20ms to 10.24s). */

#define APP_TIMER_MAX_TIMERS              6                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE           6                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL                 MSEC_TO_UNITS(500, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL                 MSEC_TO_UNITS(1000, UNIT_1_25_MS)           /**< Maximum acceptable connection interval */
#define SLAVE_LATENCY                     0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                  MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(20 * 1000)                  /**< Time from initiating event (connect or start of notification)to first time sd_ble_gap_conn_param_update is called  (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY     APP_TIMER_TICKS(5 * 1000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT      3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY            APP_TIMER_TICKS(50)                         /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define INITIAL_LLS_ALERT_LEVEL           BLE_CHAR_ALERT_LEVEL_NO_ALERT               /**< Initial value for the Alert Level characteristic in the Link Loss service. */

#define SEC_PARAM_BOND                    1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                    0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                    0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES         BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                     0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE            7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE            16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                         0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//------------------------------------------------------------------------------
// static global variables
//------------------------------------------------------------------------------
NRF_BLE_GATT_DEF(m_gatt);                                                             /**< GATT module instance. */
ble_advertising_t                         m_advertising;                              /**< Advertising module instance. BLE_ADVERTISING_DEF(m_advertising); */
static ble_dfu_t                          m_dfus;                                     /**< Structure used to identify the DFU service. */

#ifdef FREERTOS_SWITCH
#if NRF_LOG_ENABLED
static TaskHandle_t                       m_logger_thread;                            /**< Logger thread. */
#endif
TaskHandle_t                              m_advertising_start;
TaskHandle_t                              m_LED_control;
#endif

uint8_t                                   m_tbm_scan_mode = false;
uint8_t                                   m_timeslot_mode = false;
uint8_t                                   m_advertising_packet_type = 0x00;
uint8_t                                   m_eco_adv_stop = false;
ble_gap_addr_t                            m_device_addr;                              // 48-bit address, LSB format

uint8_t                                   g_is_startup;                               // startup status
uint8_t                                   g_connected;                                // connected or not connected
uint16_t                                  g_conn_handle;                              // 
uint8_t                                   g_ibeacon_mode;
ble_bms_t                                 g_bms;                                      /**< Structure to identify the Beacon Management Service. */
uint8_t                                   g_startup_stage;                            // startup status

uint32_union_t                            m_adv_count;                                /**< advertising counter */
uint32_union_t                            m_sec_count_100ms;                          /**< 100msec counter */
uint16_union_t                            m_battery_charge;                           /**< current battery charge */
uint64_union_t                            m_line_timestamp;                           /**< 64bit timestamp for LINE Beacon */
uint32_union_t                            m_tgsec_timestamp;                          /**< 32bit timestamp for Tangerine Secure iBeacon */

dectime_union_t                           m_eco_start_time;                           /**< 16bit decmal eco mode start time */
dectime_union_t                           m_eco_finish_time;                          /**< 16bit decmal eco mode finish time */

static ble_gap_adv_params_t               m_sd_adv_params;                            /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t                            m_sd_adv_handle;                            /**< Advertising handle used to identify an advertising set. */
static uint16_t                           m_enc_advdata_len;                          //
static uint8_t                            m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX]; /**< Buffer for storing an encoded advertising set. */
static uint8_t                            m_enc_advdata2[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set (iBeacon). */

uint8_t                                   m_tbm_txfrq;                               // Tangerine Beacon Management Packet Interval Counter
uint8_t                                   m_bTbm = false;

//------------------------------------------------------------------------------
// Battery monitoring
//------------------------------------------------------------------------------
nrf_saadc_value_t  m_buffer_pool[ADC_SAMPLES_IN_BUFFER];
uint8_t m_adc_status = true;

#ifdef FREERTOS_SWITCH
  //#define BATTERY_MONITORING_INTERVAL_ (12 * 60 * 60 * 1000) // 12 hours
  #define BATTERY_MONITORING_INTERVAL_ (10* 1000) // 12 hours
  static void read_battery_hysteresis(TimerHandle_t xTimer);
  static TimerHandle_t m_battery_monitoring_timer;
#else
  //#define BATTERY_MONITORING_INTERVAL APP_TIMER_TICKS(30 * 1000, APP_TIMER_PRESCALER) //  test: 30s
  #define BATTERY_MONITORING_INTERVAL APP_TIMER_TICKS(12 * 60 * 60 * 1000) // 12 hours
  APP_TIMER_DEF(m_battery_monitoring_timer_id);
  static void read_battery_hysteresis(void * p_context);
#endif


//-----------------------------------------------------------------------------
// 100msec timer count 
//-----------------------------------------------------------------------------
#ifdef FREERTOS_SWITCH
  #define COUNTER_100ms_INTERVAL_ (100) // 100msec
  static void time_100ms_count_hanlder(TimerHandle_t xTimer);
  static TimerHandle_t m_100ms_count_timer;
#else
  #define COUNTER_100ms_INTERVAL APP_TIMER_TICKS(100) // 100msec
  APP_TIMER_DEF(m_100ms_count_timer_id);
  static void time_100ms_count_hanlder(void * p_context);
#endif

//-----------------------------------------------------------------------------
// 15sec timer count for LINE Beacon 
//-----------------------------------------------------------------------------
#ifdef FREERTOS_SWITCH
  #define COUNTER_15000ms_INTERVAL_ (15000) // 15000msec = 15sec
  static void time_15000ms_count_hanlder(TimerHandle_t xTimer);
  static TimerHandle_t m_15000ms_count_timer;
#else
  #define COUNTER_15000ms_INTERVAL APP_TIMER_TICKS(15000) // 15000msec = 15sec
  APP_TIMER_DEF(m_15000ms_count_timer_id);
  static void time_15000ms_count_hanlder(void * p_context);
#endif
//-----------------------------------------------------------------------------
// 60sec timer count for startup stage.
//-----------------------------------------------------------------------------
#ifdef FREERTOS_SWITCH
  #define COUNTER_60000ms_INTERVAL_ (60000) // 60000msec = 60sec
  static void time_60000ms_count_hanlder(TimerHandle_t xTimer);
  static TimerHandle_t m_60000ms_count_timer;
#else
  #define COUNTER_60000ms_INTERVAL APP_TIMER_TICKS(60000-10) // 60000msec = 60sec
  APP_TIMER_DEF(m_60000ms_count_timer_id);
  static void time_60000ms_count_hanlder(void * p_context);
#endif

//-----------------------------------------------------------------------------
// 10min timer count for startup stage.
//-----------------------------------------------------------------------------
#ifdef FREERTOS_SWITCH
  #define COUNTER_10min_INTERVAL_ (600 * 1000) // 600000msec = 10min
  static void time_10min_count_hanlder(TimerHandle_t xTimer);
  static TimerHandle_t m_10min_count_timer;
#else
  #define COUNTER_10min_INTERVAL APP_TIMER_TICKS(600 * 1000) // 690000msec = 10min
  APP_TIMER_DEF(m_10min_count_timer_id);
  static void m_10min_count_timer(void * p_context);
#endif

//-----------------------------------------------------------------------------
// 1sec timer count for startup stage.
//-----------------------------------------------------------------------------
#ifdef FREERTOS_SWITCH
  #define COUNTER_1000ms_INTERVAL_ (1000) // 1000msec = 1sec
  static void time_1000ms_count_hanlder(TimerHandle_t xTimer);
  static TimerHandle_t m_1000ms_count_timer;
#else
  #define COUNTER_1000ms_INTERVAL APP_TIMER_TICKS(1000) // 1000msec = 1sec
  APP_TIMER_DEF(m_1000ms_count_timer_id);
  static void m_1000ms_count_timer(void * p_context);
#endif

//------------------------------------------------------------------------------
//  timelost
//------------------------------------------------------------------------------
/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0
    }
};

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void softdevice_advertising_init(uint8_t is_ibeacon)
{
  uint32_t      err_code;
  ble_advdata_t advdata;
  uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

  m_sd_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
  m_enc_advdata_len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;

  ble_advdata_manuf_data_t manuf_specific_data;
  manuf_specific_data.company_identifier = TANGERINE_COMPANY_IDENTIFIER;
 
  //manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
  manuf_specific_data.data.p_data = (uint8_t *) get_bms_info();

  manuf_specific_data.data.size   = APP_BMS_INFO_LENGTH;

  // Build and set advertising data.
  memset(&advdata, 0, sizeof(advdata));

  advdata.name_type             = BLE_ADVDATA_NO_NAME;
  advdata.flags                 = flags;
  advdata.p_manuf_specific_data = &manuf_specific_data;

  if (is_ibeacon)
  {
    err_code = ble_advdata_encode(&advdata, m_enc_advdata2, &m_enc_advdata_len);
    APP_ERROR_CHECK(err_code);
  }
  else
  {
    err_code = ble_advdata_encode(&advdata, m_enc_advdata, &m_enc_advdata_len);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_sd_adv_params, 0, sizeof(m_sd_adv_params));

    m_sd_adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    m_sd_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_sd_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_sd_adv_params.interval        = BASE_ADV_INTERVAL;
    m_sd_adv_params.duration        = 0;       // Never time out.
  }
}

/**@brief Function for handling SoC events.
 *
 * @param[in]   evt_id      Stack SoC event.
 * @param[in]   p_context   Unused.
 */
static void timeslot_soc_evt_handler(uint32_t evt_id, void * p_context)
{
  timeslot_on_sys_evt(evt_id);
}

/**@brief Function for handling a BeaconAdvertiser error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void timeslot_beacon_advertiser_error_handler(uint32_t nrf_error)
{
  NRF_LOG_INFO("beacon_advertiser_error_handler 0x%08x.\r\n", nrf_error);
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing Beacon advertiser.
 */
static void timeslot_beacon_adv_init(void)
{
  static ble_beacon_init_t beacon_init;
    
  beacon_init.adv_interval  = TS_ADV_INTERVAL;
  beacon_init.error_handler = timeslot_beacon_advertiser_error_handler;
    
  uint32_t err_code = sd_ble_gap_addr_get(&beacon_init.beacon_addr);
  APP_ERROR_CHECK(err_code);
  beacon_init.beacon_addr.addr[0] += 0x01;
    
  timeslot_init(&beacon_init);
}

/**@brief Set advertising data for every advertising event. */
static void timeslot_on_radio_event(bool radio_active) 
{
  static bool send_ibeacon = true;
  if (radio_active) 
  {
    memcpy(m_enc_advdata, get_bms_advertising_data(), BLE_GAP_ADV_SET_DATA_SIZE_MAX);
   /*
    if (send_ibeacon)
    {
      m_adv_data.adv_data.p_data  = m_enc_advdata2;
    }
    else
    {
      m_adv_data.adv_data.p_data  = m_enc_advdata;
    }
*/
    nrf_gpio_pin_toggle(9);
    if (m_bTbm) {
      m_bTbm = false;
      memcpy(m_enc_advdata, get_bms_advertising_data(), BLE_GAP_ADV_SET_DATA_SIZE_MAX);
    }
    else {
      //memcpy(m_enc_advdata, get_ibeacon_advertising_data(), BLE_GAP_ADV_SET_DATA_SIZE_MAX);
    }

    m_enc_advdata_len = BLE_GAP_ADV_SET_DATA_SIZE_MAX;
    m_adv_data.adv_data.p_data  = m_enc_advdata;
    m_adv_data.adv_data.len = m_enc_advdata_len;
    (void)sd_ble_gap_adv_set_configure(&m_sd_adv_handle, &m_adv_data, NULL);
    m_adv_handle = m_sd_adv_handle;
    send_ibeacon = !send_ibeacon;
  }
}

//-----------------------------------------------------------------------------
// functions
//-----------------------------------------------------------------------------

/**@brief
   save 15sec timestamp counter 
*/
static void save_15sec_timestamp() 
{
  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  _beacon_info[BINFO_15SEC_TIMESTAMP_IDX]   = m_line_timestamp.array[0];
  _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+1] = m_line_timestamp.array[1];
  _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+2] = m_line_timestamp.array[2];
  _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+3] = m_line_timestamp.array[3];
  _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+4] = m_line_timestamp.array[4];
  _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+5] = m_line_timestamp.array[5];
  _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+6] = m_line_timestamp.array[6];
  _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+7] = m_line_timestamp.array[7];
  
#ifdef NO_FLASH_SAVE_TIMESTAMP
  uint32_t err_code;
  err_code = tb_manager_settings_store();
  if ( err_code != NRF_SUCCESS ) NRF_LOG_INFO("tb_manager_settings_store ERROR = %d", err_code);
  APP_ERROR_CHECK(err_code);
#endif
}

/**@brief
   restore 15sec timestamp counter 
*/
static void restore_15sec_timestamp() 
{
  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  m_line_timestamp.array[0] = _beacon_info[BINFO_15SEC_TIMESTAMP_IDX];
  m_line_timestamp.array[1] = _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+1];
  m_line_timestamp.array[2] = _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+2];
  m_line_timestamp.array[3] = _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+3];
  m_line_timestamp.array[4] = _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+4];
  m_line_timestamp.array[5] = _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+5];
  m_line_timestamp.array[6] = _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+6];
  m_line_timestamp.array[7] = _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+7];
}

uint8_t rot_mm[4] = { 0x00, 0x00, 0x00, 0x00 };
static void calc_rotmm_save_15sec_tgsec_timestamp() 
{
  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  _beacon_info[BINFO_TGSECB_TIMESTAMP_IDX]   = m_tgsec_timestamp.array[0];
  _beacon_info[BINFO_TGSECB_TIMESTAMP_IDX+1] = m_tgsec_timestamp.array[1];
  _beacon_info[BINFO_TGSECB_TIMESTAMP_IDX+2] = m_tgsec_timestamp.array[2];
  _beacon_info[BINFO_TGSECB_TIMESTAMP_IDX+3] = m_tgsec_timestamp.array[3];  

  calc_store_rot_mm(rot_mm);
  _beacon_info[BINFO_TGSECB_ROTATED_BEACONID_IDX]   = rot_mm[0];
  _beacon_info[BINFO_TGSECB_ROTATED_BEACONID_IDX+1] = rot_mm[1];
  _beacon_info[BINFO_TGSECB_ROTATED_BEACONID_IDX+2] = rot_mm[2];
  _beacon_info[BINFO_TGSECB_ROTATED_BEACONID_IDX+3] = rot_mm[3];      

#ifdef NO_FLASH_SAVE_TIMESTAMP
  uint32_t err_code;
  err_code = tb_manager_settings_store();
  if ( err_code != NRF_SUCCESS ) NRF_LOG_INFO("tb_manager_settings_store ERROR = %d", err_code);
  APP_ERROR_CHECK(err_code);
#endif
}

/**@brief
   restore 15sec timestamp counter 
*/
static void restore_15sec_tgsec_timestamp() 
{
  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  m_tgsec_timestamp.array[0] = _beacon_info[BINFO_TGSECB_TIMESTAMP_IDX];
  m_tgsec_timestamp.array[1] = _beacon_info[BINFO_TGSECB_TIMESTAMP_IDX+1];
  m_tgsec_timestamp.array[2] = _beacon_info[BINFO_TGSECB_TIMESTAMP_IDX+2];
  m_tgsec_timestamp.array[3] = _beacon_info[BINFO_TGSECB_TIMESTAMP_IDX+3];    
}

/**@brief 
   100msec Interval Timer*
 */
#ifdef FREERTOS_SWITCH
static void time_100ms_count_hanlder(TimerHandle_t xTimer)
{
  UNUSED_PARAMETER(xTimer);
#else
static void time_100ms_count_hanlder(void * p_context)
{
  UNUSED_PARAMETER(p_context);
#endif
  NRF_LOG_INFO("time_100ms_count_hanlder");

  m_sec_count_100ms.value++;
}

/**@brief 
   15see Interval Timer
 */
#ifdef FREERTOS_SWITCH
static void time_15000ms_count_hanlder(TimerHandle_t xTimer)
{
  UNUSED_PARAMETER(xTimer);
#else
static void time_15000ms_count_hanlder(void * p_context)
{
  UNUSED_PARAMETER(p_context);
#endif
  NRF_LOG_INFO("time_15000ms_count_hanlder");

  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  if ( g_startup_stage == 1 ||
       _beacon_info[BINFO_STATUS_VALUE_IDX] != 4) {
    return;
  }

  if (ble_line_beacon_enablep() == 1) {
    m_line_timestamp.value++;
    save_15sec_timestamp();
    if (ble_bms_get_timeslot_status() != 0x00) {
      set_line_beacon_packet();
      set_line_ibeacon_packet();
    }
  }
  if (ble_tgsec_ibeacon_enablep() == 1) {
    m_tgsec_timestamp.value++;
    calc_rotmm_save_15sec_tgsec_timestamp();
    //if (ble_bms_get_timeslot_status() != 0x00) set_ibeacon_packet();
  }
}

/**@brief 
   60see Interval Timer
 */
#ifdef FREERTOS_SWITCH
static void time_60000ms_count_hanlder(TimerHandle_t xTimer)
{
  UNUSED_PARAMETER(xTimer);
#else
static void time_60000ms_count_hanlder(void * p_context)
{
  UNUSED_PARAMETER(p_context);
#endif
  NRF_LOG_INFO("time_60000ms_count_hanlder");

  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  if (_beacon_info[BINFO_STATUS_VALUE_IDX] != 0x00) {
    g_startup_stage = 0;
  #ifdef FREERTOS_SWITCH
    xTimerStop(xTimer, 0);
    xTimerDelete(xTimer, 0);
  #else
    app_timer_stop(m_60000ms_count_timer_id);
  #endif
  }

#if defined(LED_ENABLED)
  if (_beacon_info[BINFO_STATUS_VALUE_IDX] == 0x00) {
    execute_pending_led(LED_ON);
  }
  else {
    execute_pending_led(LED_OFF);
  }
#endif

  // Timeslot Started
  if (ble_bms_get_timeslot_status() != 0x00 &&  ble_line_beacon_enablep() == 1) {

    sd_ble_gap_adv_stop(m_sd_adv_handle);
    m_sd_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
    advertising_init(BLE_ADV_MODE_IBEACON);
    sd_ble_gap_adv_set_configure(&m_sd_adv_handle, &m_adv_data, &m_sd_adv_params);
    sd_ble_gap_adv_start(m_sd_adv_handle, APP_BLE_CONN_CFG_TAG);

    if (ble_line_beacon_enablep() == 1) {
      timeslot_start();
    }

    m_tbm_txfrq = 0;
    ret_code_t err_code = app_timer_start(m_1000ms_count_timer_id, COUNTER_1000ms_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
  }
}

/**@brief 
   10min Interval Timer
 */
#ifdef FREERTOS_SWITCH
static void time_10min_count_hanlder(TimerHandle_t xTimer)
{
  UNUSED_PARAMETER(xTimer);
#else
static void time_10min_count_hanlder(void * p_context)
{
  UNUSED_PARAMETER(p_context);
#endif
  NRF_LOG_INFO("time_10min_count_hanlder");

  // Battery update
  read_battery_hysteresis(NULL);

  // Date/Time read
  if ( !pcf8563_read() ) return;

  // RTC Chec
  if ( m_pre_time.years == 0x00 || m_pre_time.months == 0x00 || m_pre_time.days == 0x00 ) return;

  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  memcpy(&_beacon_info[BINFO_CURRENT_DATETIME_IDX], &m_pre_time, BINFO_CURRENT_DATETIME_SIZ);

  m_eco_start_time.dec.Hours     = _beacon_info[BINFO_ECO_MODE_START_TIME_IDX];
  m_eco_start_time.dec.Minutes   = _beacon_info[BINFO_ECO_MODE_START_TIME_IDX+1];
  m_eco_finish_time.dec.Hours    = _beacon_info[BINFO_ECO_MODE_FINISH_TIME_IDX];
  m_eco_finish_time.dec.Minutes  = _beacon_info[BINFO_ECO_MODE_FINISH_TIME_IDX+1];

  bool bEco_Mode = false;
  //if (_beacon_info[BINFO_ECO_MODE_STATUS_IDX] != 0x00)
  if (m_eco_start_time.wTime != m_eco_finish_time.wTime)
  {
    if ( m_eco_start_time.dec.Hours <= m_eco_finish_time.dec.Hours )
    {        
      if ( m_eco_start_time.dec.Hours <= m_pre_time.hours && m_pre_time.hours <= m_eco_finish_time.dec.Hours )
      {
        if ( m_eco_start_time.dec.Minutes <= m_eco_finish_time.dec.Minutes )
        {
          if ( m_eco_start_time.dec.Minutes <= m_pre_time.minutes && m_pre_time.minutes <= m_eco_finish_time.dec.Minutes )
            bEco_Mode = true;
         } else if ( m_eco_start_time.dec.Minutes <= m_pre_time.minutes || m_pre_time.minutes <= m_eco_finish_time.dec.Minutes ) 
            bEco_Mode = true;
      }
    } else {
      if ( m_eco_start_time.dec.Hours <= m_pre_time.hours || m_pre_time.hours <= m_eco_finish_time.dec.Hours )
      {
        if ( m_eco_start_time.dec.Minutes <= m_eco_finish_time.dec.Minutes )
        {
          if ( m_eco_start_time.dec.Minutes <= m_pre_time.minutes && m_pre_time.minutes <= m_eco_finish_time.dec.Minutes ) 
            bEco_Mode = true;
        } else if ( m_eco_start_time.dec.Minutes <= m_pre_time.minutes || m_pre_time.minutes <= m_eco_finish_time.dec.Minutes ) 
            bEco_Mode = true;
      }
    }
  }
  NRF_LOG_INFO("ECO mode = %d %02X:%02X", m_eco_adv_stop, m_pre_time.hours, m_pre_time.minutes);
  NRF_LOG_INFO("(%02X:%02X-%02X:%02X)", m_eco_start_time.dec.Hours, m_eco_start_time.dec.Minutes, m_eco_finish_time.dec.Hours, m_eco_finish_time.dec.Minutes);  
  return;
  m_eco_adv_stop = bEco_Mode;
}

/**@brief 
   1sec Interval Timer
 */
#ifdef FREERTOS_SWITCH
static void time_1000ms_count_hanlder(TimerHandle_t xTimer)
{
  UNUSED_PARAMETER(xTimer);
#else
static void time_1000ms_count_hanlder(void * p_context)
{
  UNUSED_PARAMETER(p_context);
#endif

  m_tbm_txfrq++;
  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  _beacon_info[BINFO_TBM_TXFRQ_VALUE_IDX] = 12;
  if ( m_tbm_txfrq >=_beacon_info[BINFO_TBM_TXFRQ_VALUE_IDX]) {
    NRF_LOG_INFO("TBM TX Frquency interval = %d", m_tbm_txfrq);
    m_tbm_txfrq = 0;
    m_bTbm = true;
  }
}



/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
  switch (event)
  {
    case NRF_PWR_MGMT_EVT_PREPARE_DFU:
      NRF_LOG_INFO("Power management wants to reset to DFU mode.");
      // YOUR_JOB: Get ready to reset into DFU mode
      //
      // If you aren't finished with any ongoing tasks, return "false" to
      // signal to the system that reset is impossible at this stage.
      //
      // Here is an example using a variable to delay resetting the device.
      break;

    default:
      // YOUR_JOB: Implement any of the other events available from the power management module:
      //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
      //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
      //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
      return true;
  }

  NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
  return true;
}

/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);


static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
  if (state == NRF_SDH_EVT_STATE_DISABLED)
  {
    // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
    nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

    //Go to system off.
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
  }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
  .handler = buttonless_dfu_sdh_state_observer,
};

static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
  memset(p_config, 0, sizeof(ble_adv_modes_config_t));

  p_config->ble_adv_fast_enabled  = true;
  p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
  p_config->ble_adv_fast_timeout  = APP_ADV_DURATION;
}


static void disconnect(uint16_t conn_handle, void * p_context)
{
  UNUSED_PARAMETER(p_context);

  ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
  if (err_code != NRF_SUCCESS)
  {
    NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
  }
  else
  {
    NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
  }
}


// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
  switch (event)
  {
    case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
    {
      NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

      // Prevent device from advertising on disconnect.
      ble_adv_modes_config_t config;
      advertising_config_get(&config);
      config.ble_adv_on_disconnect_disabled = true;
      //ble_advertising_modes_config_set(&m_advertising, &config);

      // Disconnect all other bonded devices that currently are connected.
      // This is required to receive a service changed indication
      // on bootup after a successful (or aborted) Device Firmware Update.
      uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
      NRF_LOG_INFO("Disconnected %d links.", conn_count);
      break;
    }

    case BLE_DFU_EVT_BOOTLOADER_ENTER:
      // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
      //           by delaying reset by reporting false in app_shutdown_handler
      NRF_LOG_INFO("Device will enter bootloader mode.");
      break;

    case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
      NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
      // YOUR_JOB: Take corrective measures to resolve the issue
      //           like calling APP_ERROR_CHECK to reset the device.
      break;

    case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
      NRF_LOG_ERROR("Request to send a response to client failed.");
      // YOUR_JOB: Take corrective measures to resolve the issue
      //           like calling APP_ERROR_CHECK to reset the device.
      APP_ERROR_CHECK(false);
      break;

    default:
      NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
      break;
  }
}

/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler_(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
  char str[256];
  sprintf(str, "Line = %d  file = %s", line_num, p_file_name);
  NRF_LOG_INFO("Line = %d  file = %s", line_num, p_file_name);
  execute_error_led(LED_ON);
  execute_led(LED_OFF);
  // FIXME NVIC_SystemReset();
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
  NRF_LOG_INFO("Line = %d  file = %s", line_num, p_file_name);
  app_error_handler_(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
  //execute_led(PIC_LED_RED);
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
  // Initialize timer module.
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);

#ifdef FREERTOS_SWITCH
  // Create counter 100msec  timer.
  m_100ms_count_timer         = xTimerCreate("COUNTER_100ms_INTERVAL",
                                        COUNTER_100ms_INTERVAL_,
                                        pdTRUE,
                                        NULL,
                                        time_100ms_count_hanlder);

  // Create counter 15000msec  timer for LINE Beacon.
  m_15000ms_count_timer       = xTimerCreate("COUNTER_15000ms_INTERVAL",
                                        COUNTER_15000ms_INTERVAL_,
                                        pdTRUE,
                                        NULL,
                                        time_15000ms_count_hanlder);

  // Create counter 60000msec  timer for power-on startup stage.
  m_60000ms_count_timer       = xTimerCreate("COUNTER_60000ms_INTERVAL",
                                        COUNTER_60000ms_INTERVAL_,
                                        pdTRUE,
                                        NULL,
                                        time_60000ms_count_hanlder);


  // Create Battery check timer for power-on startup stage.
  #if defined(BATTERY_CHECKING_ENABLED)
  //m_battery_monitoring_timer  = xTimerCreate("BATTERY_MONITORING_INTERVAL",
  //                                      BATTERY_MONITORING_INTERVAL_,
  //                                      pdTRUE,
  //                                      NULL,
  //                                      read_battery_hysteresis);
  #endif

  // Create counter 10min timer for power-on startup stage.
  #if defined(ECOMODE_CHECKING_ENABLED)
  m_10min_count_timer         = xTimerCreate("COUNTER_10min_INTERVAL",
                                        COUNTER_10min_INTERVAL_,
                                        pdTRUE,
                                        NULL,
                                        time_10min_count_hanlder);
  #endif

  /* Error checking */
  if ( (NULL == m_100ms_count_timer)
    || (NULL == m_15000ms_count_timer)
    || (NULL == m_60000ms_count_timer)
  #if defined(BATTERY_CHECKING_ENABLED)
  //|| (NULL == m_battery_monitoring_timer)
  #endif
    || (NULL == m_10min_count_timer) )
  {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }

#else
  // Create counter 100msec  timer.
  err_code = app_timer_create(&m_100ms_count_timer_id,
                              APP_TIMER_MODE_REPEATED,
                              time_100ms_count_hanlder);
  APP_ERROR_CHECK(err_code);

  // Create counter 15000msec  timer for LINE Beacon.
  err_code = app_timer_create(&m_15000ms_count_timer_id,
                              APP_TIMER_MODE_REPEATED,
                              time_15000ms_count_hanlder);
  APP_ERROR_CHECK(err_code);

  // Create counter 60000msec  timer for power-on startup stage.
  err_code = app_timer_create(&m_60000ms_count_timer_id,
                              APP_TIMER_MODE_SINGLE_SHOT,
                              time_60000ms_count_hanlder);
  APP_ERROR_CHECK(err_code);

  #if defined(BATTERY_CHECKING_ENABLED)
  //err_code = app_timer_create(&m_battery_monitoring_timer_id,
  //                            APP_TIMER_MODE_REPEATED,
  //                            read_battery_hysteresis);
  //APP_ERROR_CHECK(err_code);
  #endif

  // Create 10min  check timer for power-on startup stage.
  err_code = app_timer_create(&m_10min_count_timer_id,
                              APP_TIMER_MODE_REPEATED,
                              time_10min_count_hanlder);
  APP_ERROR_CHECK(err_code);

  // Create 1000msec  check timer for power-on startup stage.
  err_code = app_timer_create(&m_1000ms_count_timer_id,
                              APP_TIMER_MODE_REPEATED,
                              time_1000ms_count_hanlder);
  APP_ERROR_CHECK(err_code);
#endif

}

/**@brief   Function for starting application timers.
 * @details Timers are run after the scheduler has started.
 */
static void application_timers_start(void)
{
  // Start application timers.
  uint32_t                err_code;

#ifdef FREERTOS_SWITCH
  // start counter 100msec  timer.
  if (ble_eddystone_tlm_enablep() == 1) {
    if (pdPASS != xTimerStart(m_100ms_count_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
  }
  // start counter 15000ms timer.
  //if (ble_line_beacon_enablep() == 1 || ble_tgsec_ibeacon_enablep() == 1) {
    if (pdPASS != xTimerStart(m_15000ms_count_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
  //}

  // start counter 60000ms timer for power-on stage.
  if (pdPASS != xTimerStart(m_60000ms_count_timer, OSTIMER_WAIT_FOR_QUEUE))
  {
       APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
  #if defined(BATTERY_CHECKING_ENABLED)
  //if (pdPASS != xTimerStart(m_battery_monitoring_timer, OSTIMER_WAIT_FOR_QUEUE))
  //{
  //    APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  //}
  #endif
  #if defined(ECOMODE_CHECKING_ENABLED)
    uint8_t *_beacon_info = ble_bms_get_beacon_info();
    m_eco_start_time.dec.Hours     = _beacon_info[BINFO_ECO_MODE_START_TIME_IDX];
    m_eco_start_time.dec.Minutes   = _beacon_info[BINFO_ECO_MODE_START_TIME_IDX+1];
    m_eco_finish_time.dec.Hours    = _beacon_info[BINFO_ECO_MODE_FINISH_TIME_IDX];
    m_eco_finish_time.dec.Minutes  = _beacon_info[BINFO_ECO_MODE_FINISH_TIME_IDX+1];

    if (pdPASS != xTimerStart(m_10min_count_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
  #endif
#else
  // start counter 100msec  timer.
  if (ble_eddystone_tlm_enablep() == 1) {
    err_code = app_timer_start(m_100ms_count_timer_id, COUNTER_100ms_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
  }

  // start counter 15000ms timer.
  //if (ble_line_beacon_enablep() == 1 || ble_tgsec_ibeacon_enablep() == 1) {
    err_code = app_timer_start(m_15000ms_count_timer_id, COUNTER_15000ms_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
  //}

  // start counter 60000ms timer for power-on stage.
  err_code = app_timer_start(m_60000ms_count_timer_id, COUNTER_60000ms_INTERVAL, NULL);
  APP_ERROR_CHECK(err_code);

  #if defined(BATTERY_CHECKING_ENABLED)
  //err_code = app_timer_start(m_battery_monitoring_timer_id, BATTERY_MONITORING_INTERVAL, NULL);
  //APP_ERROR_CHECK(err_code);
  #endif

  #if defined(ECOMODE_CHECKING_ENABLED)
  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  m_eco_start_time.dec.Hours     = _beacon_info[BINFO_ECO_MODE_START_TIME_IDX];
  m_eco_start_time.dec.Minutes   = _beacon_info[BINFO_ECO_MODE_START_TIME_IDX+1];
  m_eco_finish_time.dec.Hours    = _beacon_info[BINFO_ECO_MODE_FINISH_TIME_IDX];
  m_eco_finish_time.dec.Minutes  = _beacon_info[BINFO_ECO_MODE_FINISH_TIME_IDX+1];

  err_code = app_timer_start(m_10min_count_timer_id, COUNTER_10min_INTERVAL, NULL);
  APP_ERROR_CHECK(err_code);
  #endif
#endif
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
  uint32_t                err_code;
  ble_gap_conn_params_t   gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

//    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_KEYRING);
//    APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency     = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

static void reset_prepare(void)
{
  uint32_t err_code;

  if (g_conn_handle != BLE_CONN_HANDLE_INVALID) {
    // Disconnect from peer.
    err_code = sd_ble_gap_disconnect(g_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    // err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    // APP_ERROR_CHECK(err_code);
  }
  else {
    // If not connected, the device will be advertising. Hence stop the advertising.
    advertising_stop();
  }

  err_code = ble_conn_params_stop();
  APP_ERROR_CHECK(err_code);

  nrf_delay_ms(500);
}

/**@brief Function for initializing the services that will be used by the application.
 */
static void services_init(void)
{
  uint32_t         err_code;
  nrf_ble_qwr_init_t        qwr_init  = {0};
  ble_dfu_buttonless_init_t dfus_init = {0};
 
  // Initialize Queued Write Module. 
  qwr_init.error_handler = nrf_qwr_error_handler;

  //err_code = ble_bms_init(&m_bms, &bms_init);
  err_code = ble_bms_init(&g_bms);
  APP_ERROR_CHECK(err_code);

  err_code = tb_manager_init();
  APP_ERROR_CHECK(err_code);

  // Firmware Versionを上書きする
  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  uint8_t firmware_version[2] = {APP_FIRMWARE_VERSION_VALUE};
  _beacon_info[BINFO_VERSION_VALUE_IDX]   = firmware_version[0];
  _beacon_info[BINFO_VERSION_VALUE_IDX+1] = firmware_version[1];

  // Timeslot Mode
  m_timeslot_mode = _beacon_info[BINFO_TIMESLOT_MODE_STATUS_IDX];
  if (m_timeslot_mode != 0x00) {
    switch(m_timeslot_mode & 0x0f) {
    case 0 :  // TGR
      m_advertising_packet_type = 0x40; break;
    case 1 :  // iBeacon + TGR
      m_advertising_packet_type = 0x01; break;
    case 2 :  // Secure iBeacon + TGR
      m_advertising_packet_type = 0x20; break;
    case 3 :  // LINE + TGR
      m_advertising_packet_type = 0x10; break;
    case 4 :  // LINE + iBeacon + TGR
      m_advertising_packet_type = 0x11; break;
    case 5 :  // LINE + Secure iBeacon + TGR
      m_advertising_packet_type = 0x30; break;
    default :
      m_advertising_packet_type = 0x40; break;
    }  
  }
  _beacon_info[BINFO_STATUS_VALUE_IDX] = 4;
  m_advertising_packet_type = 0x30;    // ##DEBUG##
  m_timeslot_mode = true;    // ##DEBUG##

  // DFU Services
#ifndef BOOTLOADER_NOT_FOUND
  dfus_init.evt_handler = ble_dfu_evt_handler;

  err_code = ble_dfu_buttonless_init(&dfus_init);
  APP_ERROR_CHECK(err_code);
#endif // DFU 

  // 初期値の電波強度
#ifdef TANGERINE_NRF52
  int8_t txPowerLevel = get_tx_power_level( _beacon_info[BINFO_TXPWR_VALUE_IDX] );
  err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, txPowerLevel);
  APP_ERROR_CHECK(err_code);
#endif
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(g_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        execute_led(LED_OFF);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
  uint32_t               err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params                  = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail             = true;
  //cp_init.evt_handler                    = on_conn_params_evt;
  cp_init.evt_handler                    = NULL;
  cp_init.error_handler                  = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
  uint32_t        err_code;

  switch (p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_CONNECTED:
    {
      g_connected = 1;
      g_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
      execute_led(LED_ON);
      break;
    }

    case BLE_GAP_EVT_DISCONNECTED:
    {
      g_connected = 0;
      g_conn_handle = BLE_CONN_HANDLE_INVALID;
      execute_led(LED_OFF);
      if (ble_bms_get_timeslot_status() == 0x00) {
        advertising_start();
      } else {
        (void)sd_ble_gap_adv_set_configure(&m_sd_adv_handle, &m_adv_data, &m_sd_adv_params);
        (void)sd_ble_gap_adv_start(m_sd_adv_handle, APP_BLE_CONN_CFG_TAG);
      }

      break;
    }

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
    {
      // Pairing not supported
      //err_code = sd_ble_gap_sec_params_reply(g_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
      //err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.common_evt.conn_handle,　BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,　NULL,　NULL);
      //APP_ERROR_CHECK(err_code);
      break;
    }

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
    {
      // No system attributes have been stored.
      err_code = sd_ble_gatts_sys_attr_set(p_ble_evt->evt.common_evt.conn_handle, NULL, 0, 0);
      APP_ERROR_CHECK(err_code);
      break;
    }

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
      NRF_LOG_DEBUG("PHY update request.");
      ble_gap_phys_t const phys =
      {
        .rx_phys = BLE_GAP_PHY_AUTO,
        .tx_phys = BLE_GAP_PHY_AUTO,
      }; 
      err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
      APP_ERROR_CHECK(err_code);
      break;
    }

    case BLE_GAP_EVT_TIMEOUT:
    {
      g_connected = 0;
      g_conn_handle = BLE_CONN_HANDLE_INVALID;
      execute_led(LED_OFF);
      if (ble_bms_get_timeslot_status() == 0x00) {
        advertising_start();
      } else {
        (void)sd_ble_gap_adv_set_configure(&m_sd_adv_handle, &m_adv_data, &m_sd_adv_params);
        (void)sd_ble_gap_adv_start(m_sd_adv_handle, APP_BLE_CONN_CFG_TAG);
      }
      break;
    }

    case BLE_GATTC_EVT_TIMEOUT:
    case BLE_GATTS_EVT_TIMEOUT:
    {
      // Disconnect on GATT Server and Client timeout events.
      err_code = sd_ble_gap_disconnect(g_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      g_connected = 0;
      g_conn_handle = BLE_CONN_HANDLE_INVALID;
      execute_led(LED_OFF);
      break;
    }

    case BLE_ADV_EVT_IDLE:
    {
      //sleep_mode_enter();
      break;
    }

    default:
    {
      // No implementation needed.
      break;
    }
  }
}

/**@brief Function for handling BLE events.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
//static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
  //dm_ble_evt_handler(p_ble_evt);
  //ble_conn_params_on_ble_evt(p_ble_evt);
  ble_bms_on_ble_evt(&g_bms, p_ble_evt);
  ble_dfu_buttonless_on_ble_evt(&m_dfus, p_ble_evt);

  on_ble_evt(p_ble_evt);
}

static void sys_evt_dispatch(uint32_t sys_evt)
{
  Fs_sys_event_handler(sys_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
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
  
  // Enable DCDC
  err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_DISABLE);
  APP_ERROR_CHECK(err_code);

  // Register a handler for sys events.
  NRF_SDH_SOC_OBSERVER(m_soc_observer, BLE_ADV_SOC_OBSERVER_PRIO, timeslot_soc_evt_handler, NULL);
  // Register handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for power management.
 */
static void power_manage(void)
{
  uint32_t err_code = sd_app_evt_wait();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
  return;
  if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
  {
    nrf_saadc_value_t adc_result;
    ret_code_t err_code;

    adc_result = p_event->data.done.p_buffer[0];

    //err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, ADC_SAMPLES_IN_BUFFER);
    err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
    APP_ERROR_CHECK(err_code);

    m_adc_status = false;
  }
}

/**@brief Function for configuring ADC to do battery level conversion.
 */
static void adc_configure(void)
{
  ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
  APP_ERROR_CHECK(err_code);

  nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
  err_code = nrf_drv_saadc_channel_init(0, &channel_config);
  APP_ERROR_CHECK(err_code);

  return;
  err_code = nrf_drv_saadc_buffer_convert(&m_buffer_pool[0], 1);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_saadc_buffer_convert(&m_buffer_pool[1], 1);
  APP_ERROR_CHECK(err_code);
  //err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool, ADC_SAMPLES_IN_BUFFER);
  //APP_ERROR_CHECK(err_code);
}

#ifdef FREERTOS_SWITCH
static void read_battery_hysteresis(TimerHandle_t xTimer)
{
  UNUSED_PARAMETER(xTimer);
#else
static void read_battery_hysteresis(void * p_context)
{
  UNUSED_PARAMETER(p_context);
#endif
  uint16_t blevel = get_battery_level();
  uint8_t bpercent = battery_level_to_percent(blevel);
  uint8_t bpercent10 = battery_level_to_percent_devidedby10(blevel);
  uint8_t *_beacon_info = ble_bms_get_beacon_info();

  // バッテリ残量 (00 ~ FF)
  _beacon_info[BINFO_BATTERY_VALUE_IDX] &= 0b00001111;

  if (bpercent <= 25) {
    // 25%
    _beacon_info[BINFO_BATTERY_VALUE_IDX] |= (1 << 4);
  } else if (bpercent <= 50) {
    // 50%
    _beacon_info[BINFO_BATTERY_VALUE_IDX] |= (1 << 5);
  } else if (bpercent <= 75) {
    // 75%
    _beacon_info[BINFO_BATTERY_VALUE_IDX] |= (1 << 4);
    _beacon_info[BINFO_BATTERY_VALUE_IDX] |= (1 << 5);
  } else if (bpercent <= 100) {
    // 100%
    _beacon_info[BINFO_BATTERY_VALUE_IDX] |= (1 << 6);
  } else {
    // 100%
    _beacon_info[BINFO_BATTERY_VALUE_IDX] |= (1 << 6);
  }

  m_battery_charge.value = blevel;

  _beacon_info[BINFO_BATTERY_LEVEL10_VALUE_IDX] = bpercent10;

  NRF_LOG_INFO("read_battery_hysteresis ADC=%d bpercend=%d bpaercent10=%d",blevel, bpercent, bpercent10 );

  // rebuild all packet data
  //build_all_data();
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);
}

/**@brief Function for handling File Data Storage events.
 *
 * @param[in] p_evt  Peer Manager event.
 * @param[in] cmd
 */
static void fds_evt_handler(fds_evt_t const * const p_evt)
{
    if (p_evt->id == FDS_EVT_GC)
    {
        NRF_LOG_DEBUG("GC completed\n");
    }
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init()
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = fds_register(fds_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/** @brief Clear bonding information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

#ifdef FREERTOS_SWITCH
#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
     vTaskResume(m_logger_thread);
#endif
}

static void tsk_Advertising_start(void * arg)
{
  UNUSED_PARAMETER(arg);

  while(true)
  {
    NRF_LOG_INFO("vTaskSuspend_tsk_Advertising_start");
    vTaskSuspend( m_advertising_start );
    advertising_start();
  }
}

static void tsk_LED_control(void * arg)
{
  UNUSED_PARAMETER(arg);

  while(true)
  {
    vTaskSuspend( NULL );
    if ( m_Blink_LED_count != 0 )
    {
      blink_led(m_Blink_LED_count);
      m_Blink_LED_count = 0;
    }
  }
}
#endif

/**@brief Function for initializing logging. */
void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing the clock.
 */
void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}



/**@brief F unction for application main entry.
 */
int main(void)
  {
  uint32_t err_code;
  bool erase_bonds;

  g_is_startup = 1;
  g_connected = 0;
  g_conn_handle = BLE_CONN_HANDLE_INVALID;
  g_ibeacon_mode = 0;
  g_startup_stage = 1;
  m_Blink_LED_count = 0;
  m_Execute_led_flash_type1 = false;

  // Initialize.
  log_init();
#ifdef FREERTOS_SWITCH
  clock_init();
  // Activate deep sleep mode.
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
#endif
  pcf8563_init(SCL_PIN_, SDA_PIN_);
  adc_configure();

#ifdef BOOTLOADER_NOT_FOUND
  // Initialize the async SVCI interface to bootloader before any interrupts are enabled.
  err_code = ble_dfu_buttonless_async_svci_init();
  APP_ERROR_CHECK(err_code);
#endif

  timers_init();
  gpiote_init();
  ble_stack_init();
  tb_manager_pstorage_init();

  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);

#ifdef FREERTOS_SWITCH
#if NRF_LOG_ENABLED
    // Start execution.
    if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 256, NULL, 1, &m_logger_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif
#endif

  gap_params_init();
  gatt_init();
  conn_params_init();
  peer_manager_init();
  services_init();
  sd_ble_gap_addr_get(&m_device_addr);

  tb_manager_get_deviceid();

  //-------------------------------------------------
  // system start....
  //-------------------------------------------------
#if defined(BATTERY_CHECKING_ENABLED)
  // 一度バッテリの状態を読み込む
  read_battery_hysteresis(NULL);
#endif

  application_timers_start();
  advertising_mode_reset();
  
#if defined(LED_ENABLED)
  blink_led(2);
  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  if (_beacon_info[BINFO_STATUS_VALUE_IDX] == 0x00) {
    execute_pending_led(LED_ON);
  }
  else {
    execute_pending_led(LED_OFF);
  }
#endif

  m_adv_count.value = 0;           
  m_sec_count_100ms.value = 0;     

  if (ble_line_beacon_enablep() == 1) {
    restore_15sec_timestamp();
  }
  if (ble_tgsec_ibeacon_enablep() == 1) {
    m_tgsec_timestamp.value = 0;
    restore_15sec_tgsec_timestamp();
  }

  // once build all advertising packet data
  build_all_data();

  // FreeRTOS Control Task Initialize
#ifdef FREERTOS_SWITCH
  #ifdef TASK_ADV_RESUME
    if (pdPASS != xTaskCreate(tsk_Advertising_start, "vAdvertising_start", 256, NULL, 1, &m_advertising_start)) 
      APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  #endif

  #ifdef TASK_LED_CONTROL
    if (pdPASS != xTaskCreate(tsk_LED_control, "LED_control", 256, NULL, 1, &m_LED_control))
      APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    m_Blink_LED_count = 2;
    vTaskResume(m_LED_control);
  #endif
#endif

#ifdef TIMESLOT_MODE
  #ifdef FREERTOS_SWITCH
    // Create a FreeRTOS task for the BLE stack.
    // The task will run advertising_start() before entering its loop.
    nrf_sdh_freertos_init(advertising_start, &erase_bonds);
  #else
    advertising_start();
  #endif
#else
  // Notification
  ble_radio_notification_init(APP_IRQ_PRIORITY_LOW, NRF_RADIO_NOTIFICATION_DISTANCE_5500US, timeslot_on_radio_event);

  // Start SoftDevice beacon,
  advertising_init(BLE_ADV_MODE_BMS);

  if (ble_bms_get_timeslot_status() != 0x00) {
    softdevice_advertising_init(false);   // Initialize Nordic beacon
    softdevice_advertising_init(true);    // Initialize iBeacon
    timeslot_beacon_adv_init();           // Initialize TimeSlot beacon

    m_adv_data.adv_data.p_data  = m_enc_advdata;
    m_adv_data.adv_data.len     = m_enc_advdata_len;
    (void)sd_ble_gap_adv_set_configure(&m_sd_adv_handle, &m_adv_data, &m_sd_adv_params);
    (void)sd_ble_gap_adv_start(m_sd_adv_handle, APP_BLE_CONN_CFG_TAG);
    m_adv_handle = m_sd_adv_handle;
  }
  else {
    advertising_start();
  }
#endif
  // Start execution.
    NRF_LOG_INFO("Tangerin Beacon started.");

#ifdef FREERTOS_SWITCH
  // Start FreeRTOS scheduler.
  #if NRF_LOG_ENABLED
    tracetask_write_switch(1);
  #endif
  vTaskStartScheduler();
#endif

  // Should not reach here.
  for (;;)
  {
  #ifdef FREERTOS_SWITCH
    APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
  #else
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    NRF_LOG_FLUSH();
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
  #ifdef CONSUMPTION_CURRENT_EXAMINATION
    power_manage();
  #endif
#endif
  }
}