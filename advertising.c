#define  _ADVERTISING_CODE_

#include "advertising.h"
#include "app_timer.h"

//app_timer_id_t                g_adv_switch_timer_id;                          /**< Advertising Switch Timer. */

#ifndef ADV_SWITCH_TIMER_APP_TIMER
  APP_TIMER_DEF(g_adv_switch_timer_id);
#endif

static ble_advertising_mode_t g_advertising_mode;                             /**< Variable to keep track of when we are */
static uint8_t                g_current_mode_index;

uint8_t                       m_adv_debug = 0;
uint8_t                       m_adv_debug2 = 0;
uint8_t                       m_adv_init = false;
uint8_t                       m_adv_switch_timer_start = false;
ble_gap_adv_params_t          m_adv_params;                                   /**< Parameters to be passed to the stack when starting advertising. */
uint8_t                       m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;  /**< Advertising handle used to identify an advertising set. */
bool                          m_adv_switch_timer_enabled;                     /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */

#ifdef  FREERTOS_SWITCH
  TimerHandle_t                 m_adv_switch_timer;                             /**< Definition of battery timer. */
#endif

ble_advertising_mode_t get_current_advmode()
{
  return g_advertising_mode;
}

void advertising_mode_reset()
{
  g_advertising_mode = BLE_ADV_MODE_BMS;
  g_current_mode_index = 0;
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 *
 * @param[in]  adv_flags  Indicates which type of advertisement to use, see @ref BLE_GAP_DISC_MODES.
 *
 */
void advertising_init(ble_advertising_mode_t mode)
{
  if (mode == BLE_ADV_MODE_IBEACON) {
    ibeacon_advertising_init();
  }
  else if (mode == BLE_ADV_MODE_EDDYSTONE_URL1) {
    eddtystone_advertising_init(BLE_ADV_MODE_EDDYSTONE_URL1);
  }
  else if (mode == BLE_ADV_MODE_EDDYSTONE_URL2) {
    eddtystone_advertising_init(BLE_ADV_MODE_EDDYSTONE_URL2);
  }
  else if (mode == BLE_ADV_MODE_EDDYSTONE_UID) {
    eddtystone_advertising_init(BLE_ADV_MODE_EDDYSTONE_UID);
  }
  else if (mode == BLE_ADV_MODE_EDDYSTONE_TLM) {
    eddtystone_advertising_init(BLE_ADV_MODE_EDDYSTONE_TLM);
  }
  else if (mode == BLE_ADV_MODE_LINE_IBEACON) {
    line_ibeacon_advertising_init();
  }
  else if (mode == BLE_ADV_MODE_LINE_PACKET) {
    line_beacon_packet_advertising_init();
  }
  else if (mode == BLE_ADV_MODE_PAUSE) {
    // NOP
  }
  
  else {
    bms_advertising_init(g_bms);
    m_adv_init = true;
  }

  g_is_startup = 0;
}

/**@brief Function for starting advertising.
 */
void advertising_start(void)
{
  uint32_t             err_code;
  ble_gap_adv_params_t adv_params;
  ble_gap_adv_data_t * p_adv_data = NULL;

  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  uint8_t txFreq = _beacon_info[BINFO_TXFRQ_VALUE_IDX];
  uint16_t txFrequencyValue = get_beacon_frequency(txFreq, TIME_UNIT_0625);
  uint16_t txFrequencyValue_msec = get_beacon_frequency(txFreq, TIME_UNIT_MSEC);
  uint16_t timeout = txFrequencyValue_msec;

  // Initialize advertising parameters with defaults values
  memset(&adv_params, 0, sizeof(adv_params));

  adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED; //BLE_GAP_ADV_TYPE_ADV_IND
  adv_params.p_peer_addr = NULL;
  adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;

  // support frequency less than 100 msec 
  if (txFrequencyValue_msec < 100) {
    txFrequencyValue = BLE_GAP_ADV_INTERVAL_MIN;  //BLE_GAP_ADV_NONCON_INTERVAL_MIN;
    timeout = 100;
  }

  adv_params.interval    = txFrequencyValue + ADV_INTERVAL_MARGIN;
  adv_params.duration    = timeout + ADV_TIMEOUT_MARGIN;

  if (g_ibeacon_mode == 1 
      || g_advertising_mode == BLE_ADV_MODE_IBEACON 
      || g_advertising_mode == BLE_ADV_MODE_EDDYSTONE_URL1
      || g_advertising_mode == BLE_ADV_MODE_EDDYSTONE_URL2
      || g_advertising_mode == BLE_ADV_MODE_EDDYSTONE_UID
      || g_advertising_mode == BLE_ADV_MODE_EDDYSTONE_TLM
      || g_advertising_mode == BLE_ADV_MODE_LINE_IBEACON
      || g_advertising_mode == BLE_ADV_MODE_LINE_PACKET)
    {
      adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED; //BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    }

  // Start advertising.
#ifdef TIMESLOT_DEBUG
  if (ble_bms_get_timeslot_status() == 0x00) start_adv_switch_timer(txFreq);
#else
  start_adv_switch_timer(txFreq);
#endif

  if (_beacon_info[BINFO_ECO_MODE_STATUS_IDX] == 0x00 || !m_eco_adv_stop) {   
    if (g_advertising_mode != BLE_ADV_MODE_PAUSE) {  

#ifdef TIMESLOT_DEBUG
      if (ble_bms_get_timeslot_status() != 0x00) {
        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
      } else {
#endif
        err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
        APP_ERROR_CHECK(err_code);
#ifdef TIMESLOT_DEBUG
      }
#endif
    }
  }

  if (g_advertising_mode == BLE_ADV_MODE_EDDYSTONE_URL1
      || g_advertising_mode == BLE_ADV_MODE_EDDYSTONE_URL2
      || g_advertising_mode == BLE_ADV_MODE_EDDYSTONE_UID
      || g_advertising_mode == BLE_ADV_MODE_EDDYSTONE_TLM)
    {
      m_adv_count.value++;
    }
}

void advertising_start_task_resume(void)
{
#ifdef TASK_ADV_RESUME
  if (__get_IPSR() != 0) {
    xTaskResumeFromISR( m_advertising_start );
  } else {
    vTaskResume( m_advertising_start );
  }
#endif
}


void advertising_stop(void)
{
  uint32_t  err_code;

  stop_adv_switch_timer();

  err_code = sd_ble_gap_adv_stop(m_adv_handle);
  //APP_ERROR_CHECK(err_code);
}

void advertising_restart(void)
{
  uint32_t  err_code;
  err_code = sd_ble_gap_adv_stop(m_adv_handle);
  APP_ERROR_CHECK(err_code);

  advertising_start();
}

/**@brief stop adv_switch_timer 
*/
void stop_adv_switch_timer() 
{
#ifndef ADV_SWITCH_TIMER_APP_TIMER
  app_timer_stop(g_adv_switch_timer_id);
#endif
}

/**@brief start adv_switch_timer 
*/
void start_adv_switch_timer(uint8_t txFreq)
{
  uint32_t             err_code;

  uint16_t txFrequencyValue = get_beacon_frequency(txFreq, TIME_UNIT_MSEC); 

#ifndef ADV_SWITCH_TIMER_APP_TIMER
  // Create adv_switch timer.
  if (!m_adv_switch_timer_start) {
    err_code = app_timer_create(&g_adv_switch_timer_id, APP_TIMER_MODE_REPEATED, adv_switch_handler);
    APP_ERROR_CHECK(err_code);
    m_adv_switch_timer_start = true;
  }
  uint32_t adv_switch_timer_ticks  =  APP_TIMER_TICKS(txFrequencyValue);
  err_code = app_timer_start(g_adv_switch_timer_id, adv_switch_timer_ticks, NULL);
  APP_ERROR_CHECK(err_code);
#else
  if (!m_adv_switch_timer_start) {
   // Create adv_switch timer.
    m_adv_switch_timer = xTimerCreate("ADV_SWITCH_TIMER", txFrequencyValue, pdTRUE, NULL, adv_switch_handler);
    m_adv_switch_timer_start = true;
  }
  if (__get_IPSR() != 0) {
    BaseType_t yieldReq = pdFALSE;
    if (xTimerChangePeriodFromISR(m_adv_switch_timer, txFrequencyValue, &yieldReq) != pdPASS) APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    if ( xTimerStartFromISR(m_adv_switch_timer, &yieldReq) != pdPASS ) APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    portYIELD_FROM_ISR(yieldReq);
  } else {
    if (xTimerChangePeriod(m_adv_switch_timer, txFrequencyValue, OSTIMER_WAIT_FOR_QUEUE) != pdPASS) APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    if (xTimerStart(m_adv_switch_timer, OSTIMER_WAIT_FOR_QUEUE) != pdPASS) APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
  }
#endif
}

/**@brief adv_switch_handler
*/
#ifdef ADV_SWITCH_TIMER_APP_TIMER
void adv_switch_handler(TimerHandle_t xTimer)
{
  UNUSED_PARAMETER(xTimer);
#else
void adv_switch_handler(void * p_context)
{
  UNUSED_PARAMETER(p_context);
#endif
  nrf_gpio_pin_toggle(DEBUG_PIN);

  // HOSTと接続中の場合、Adv Switchは行わない
  if (g_connected == 1) {
    return;
  }

  // power-on starup stage
  if (g_startup_stage == 1) {
    return;
  }

  uint8_t *_beacon_info = ble_bms_get_beacon_info();

#if defined(LED_ENABLED)
  if (_beacon_info[BINFO_STATUS_VALUE_IDX] == 0x00) {
    execute_pending_led(LED_ON);
  }
  else {
    execute_pending_led(LED_OFF);
  }
#endif

  /* stop advertising and timer */
#ifdef TIMESLOT_DEBUG
  if (ble_bms_get_timeslot_status() == 0x00) advertising_stop();
#endif
  advertising_stop();
#ifdef TIMESLOT_DEBUG
  sd_radio_session_close();
#endif

  if (g_ibeacon_mode == 1) {
    // iBeacon互換動作モードの場合、強制的に常にiBeacon送信
    g_advertising_mode = BLE_ADV_MODE_IBEACON;
  }
  else {

    bool is_select = false;
    while (is_select == false) {

      ble_advertising_mode_t current_mode = _beacon_info[BINFO_MODE_LIST_IDX+g_current_mode_index];
      if (current_mode == BLE_ADV_MODE_UNDEFINED) {
        current_mode = BLE_ADV_MODE_BMS;
      }

      switch (current_mode) {

      case BLE_ADV_MODE_IBEACON:   // Tangerine iBeacon
        
        if (ble_ibeacon_enablep() == 1 | ble_tgsec_ibeacon_enablep() == 1) {
          if ((_beacon_info[BINFO_STATUS_VALUE_IDX] == 1 // 登録済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 2 // 初期設定済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 3 // 再初期設定済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 4 // アクティブ
               ) && _beacon_info[BINFO_TXPWR_VALUE_IDX] > 0 // 電波強度が1以上の設定
              ) {
            g_advertising_mode = BLE_ADV_MODE_IBEACON;
            is_select = true;
          }
        }
        break;

      case BLE_ADV_MODE_EDDYSTONE_URL1: // EddyStone-URL 1
  
        if (ble_eddystone_url1_enablep() == 1) {
          if ((_beacon_info[BINFO_STATUS_VALUE_IDX] == 1 // 登録済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 2 // 初期設定済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 3 // 再初期設定済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 4 // アクティブ
               ) && _beacon_info[BINFO_TXPWR_VALUE_IDX] > 0 // 電波強度が1以上の設定
              ) {
            g_advertising_mode = BLE_ADV_MODE_EDDYSTONE_URL1;
            is_select = true;
          }
        }
        break;

      case BLE_ADV_MODE_EDDYSTONE_URL2:

        if (ble_eddystone_url2_enablep() == 1) {
          if ((_beacon_info[BINFO_STATUS_VALUE_IDX] == 1 // 登録済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 2 // 初期設定済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 3 // 再初期設定済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 4 // アクティブ
               ) && _beacon_info[BINFO_TXPWR_VALUE_IDX] > 0 // 電波強度が1以上の設定
              ) {
            g_advertising_mode = BLE_ADV_MODE_EDDYSTONE_URL2;
            is_select = true;
          }
        }
        break;

      case BLE_ADV_MODE_EDDYSTONE_UID:

        if (ble_eddystone_uid_enablep() == 1) {
          if ((_beacon_info[BINFO_STATUS_VALUE_IDX] == 1 // 登録済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 2 // 初期設定済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 3 // 再初期設定済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 4 // アクティブ
               ) && _beacon_info[BINFO_TXPWR_VALUE_IDX] > 0 // 電波強度が1以上の設定
              ) {
            g_advertising_mode = BLE_ADV_MODE_EDDYSTONE_UID;
            is_select = true;
          }
        }
        break;

      case BLE_ADV_MODE_EDDYSTONE_TLM:

        if (ble_eddystone_tlm_enablep() == 1) {
          if ((_beacon_info[BINFO_STATUS_VALUE_IDX] == 1 // 登録済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 2 // 初期設定済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 3 // 再初期設定済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 4 // アクティブ
               ) && _beacon_info[BINFO_TXPWR_VALUE_IDX] > 0 // 電波強度が1以上の設定
              ) {
            g_advertising_mode = BLE_ADV_MODE_EDDYSTONE_TLM;
            is_select = true;
          }
        }
        break;

      case BLE_ADV_MODE_LINE_IBEACON:   // LINE iBeacon

        if (ble_line_beacon_enablep() == 1) {
          if ((_beacon_info[BINFO_STATUS_VALUE_IDX] == 1 // 登録済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 2 // 初期設定済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 3 // 再初期設定済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 4 // アクティブ
               ) && _beacon_info[BINFO_TXPWR_VALUE_IDX] > 0 // 電波強度が1以上の設定
              ) {
            g_advertising_mode = BLE_ADV_MODE_LINE_IBEACON;
            is_select = true;
          }
        }
        break;

      case BLE_ADV_MODE_LINE_PACKET:

        if (ble_line_beacon_enablep() == 1) {
          if ((_beacon_info[BINFO_STATUS_VALUE_IDX] == 1 // 登録済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 2 // 初期設定済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 3 // 再初期設定済み
               || _beacon_info[BINFO_STATUS_VALUE_IDX] == 4 // アクティブ
               ) && _beacon_info[BINFO_TXPWR_VALUE_IDX] > 0 // 電波強度が1以上の設定
              ) {
            g_advertising_mode = BLE_ADV_MODE_LINE_PACKET;
            is_select = true;
          }
        }
        break;

      case BLE_ADV_MODE_PAUSE: 

        g_advertising_mode = BLE_ADV_MODE_PAUSE;
        is_select = true;
        break;

      default: // BLE_ADV_MODE_BMS:
      
        g_advertising_mode = BLE_ADV_MODE_BMS;
        is_select = true;

      }

      // current mode forwarding
      g_current_mode_index++;
      if (g_current_mode_index > BINFO_MODE_LIST_SIZ || _beacon_info[BINFO_MODE_LIST_IDX+g_current_mode_index] == BLE_ADV_MODE_UNDEFINED) {
        g_current_mode_index = 0;
      }

    } // END OF while(is_select == false)
    
    advertising_init( g_advertising_mode );

  } // END OF IF ELSE

  /* start adv_switch_timer and advertise */
#ifdef TIMESLOT_DEBUG
  if (ble_bms_get_timeslot_status() == 0x00) advertising_start();  
#else
  advertising_start();  
#endif

#ifdef TIMESLOT_DEBUG
  // Timeslot Started
  if (ble_bms_get_timeslot_status() != 0x00) {
    uint32_t err_code = timeslot_start();
    APP_ERROR_CHECK(err_code);
  }
#endif
}


// Beacon Frequency
uint16_t get_beacon_frequency(uint8_t tx_freq, uint8_t unit) 
{
  if (g_ibeacon_mode == 1) {
    // iBeacon互換動作モードの場合、電波送信頻度は100ms固定とする
    return APP_ADV_INTERVAL_0000;
  }

  if (unit == TIME_UNIT_MSEC) {

    switch( tx_freq ) {
    case 0:
      return APP_ADV_INTERVAL_0000_MSEC;
    case 1:
      return APP_ADV_INTERVAL_0001_MSEC;
    case 2:
      return APP_ADV_INTERVAL_0010_MSEC;
    case 3:
      return APP_ADV_INTERVAL_0011_MSEC;
    case 4:
      return APP_ADV_INTERVAL_0100_MSEC;
    case 5:
      return APP_ADV_INTERVAL_0101_MSEC;
    case 6:
      return APP_ADV_INTERVAL_0110_MSEC;
    case 7:
      return APP_ADV_INTERVAL_0111_MSEC;
    case 0xFF:
      return APP_ADV_INTERVAL_0010_MSEC;
    case 8:
      return APP_ADV_INTERVAL_1000_MSEC;
    case 9:
      return APP_ADV_INTERVAL_1001_MSEC;
    case 10:
      return APP_ADV_INTERVAL_1010_MSEC;
    case 11:
      return APP_ADV_INTERVAL_1011_MSEC;
    case 12:
      return APP_ADV_INTERVAL_1100_MSEC;
    case 13:
      return APP_ADV_INTERVAL_1101_MSEC;
    case 14:
      return APP_ADV_INTERVAL_1110_MSEC;
    case 15:
      return APP_ADV_INTERVAL_1111_MSEC;
    }
    return APP_ADV_INTERVAL_0010_MSEC;

  } 
  else  { // unit == TIME_UNIT_0625

    switch( tx_freq ) {
    case 0:
      return APP_ADV_INTERVAL_0000;
    case 1:
      return APP_ADV_INTERVAL_0001;
    case 2:
      return APP_ADV_INTERVAL_0010;
    case 3:
      return APP_ADV_INTERVAL_0011;
    case 4:
      return APP_ADV_INTERVAL_0100;
    case 5:
      return APP_ADV_INTERVAL_0101;
    case 6:
      return APP_ADV_INTERVAL_0110;
    case 7:
      return APP_ADV_INTERVAL_0111;
    case 0xFF:
      return APP_ADV_INTERVAL_0010;

    case 8:
      return APP_ADV_INTERVAL_1000;
    case 9:
      return APP_ADV_INTERVAL_1001;
    case 10:
      return APP_ADV_INTERVAL_1010;
    case 11:
      return APP_ADV_INTERVAL_1011;
    case 12:
      return APP_ADV_INTERVAL_1100;
    case 13:
      return APP_ADV_INTERVAL_1101;
    case 14:
      return APP_ADV_INTERVAL_1110;
    case 15:
      return APP_ADV_INTERVAL_1111;
    }
    return APP_ADV_INTERVAL_0010;

  }
}

void build_all_data()
{
  build_eddystone_uid_data();
  build_eddystone_url_data();
  build_ibeacon_data();
  build_bms_data();
  build_line_ibeacon_data();
  build_line_beacon_packet_servicedata();
}
