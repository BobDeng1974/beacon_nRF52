#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nordic_common.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_soc.h"

#include "ble_bms.h"
#include "advertising.h"

static void bms_data_handler(ble_bms_t *p_bms, uint8_t *p_data, uint16_t length, uint16_t handle);
static bool ac_verify(uint8_t ac_key, uint8_t ac_match);

/**< Flash handle where discovered services for bonded masters should be stored. */
//static pstorage_handle_t     m_flash_handle;

/**@brief Function for preparing the reset, disabling SoftDevice and jump to the bootloader.
 */
static void bootloader_start(void)
{
    ble_dfu_buttonless_bootloader_start_finalize();
}


/**@brief     Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the S110 SoftDevice.
 *
 * @param[in] p_bms     Beacon Management Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_bms_t *p_bms, ble_evt_t *p_ble_evt)
{
    p_bms->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief     Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the S110
 *            SoftDevice.
 *
 * @param[in] p_bms     Beacon Management Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_bms_t *p_bms, ble_evt_t *p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_bms->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief     Function for handling the @ref BLE_GATTS_EVT_WRITE event from the S110 SoftDevice.
 *
 * @param[in] p_dfu     Beacon Management Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_bms_t *p_bms, ble_evt_t *p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (p_bms->data_handler != NULL)
    {
        p_bms->data_handler(p_bms, p_evt_write->data, p_evt_write->len, p_evt_write->handle);
    }
    else
    {
        // Do Nothing. This event is not relevant to this service.
    }
}


/**@brief       Function for adding settings characteristic.
 *
 * @param[in]   p_bms        Beacon Management Service structure.
 * @param[in]   p_bms_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t settings_char_add(ble_bms_t * p_bms, const ble_bms_init_t * p_bms_init)
{
    /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

//    char_md.char_props.notify = 1;
    char_md.char_props.write            = 1;
    char_md.char_props.write_wo_resp    = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type             = p_bms->uuid_type;
    ble_uuid.uuid             = BLE_UUID_BMS_SETTINGS_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc              = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth           = 0;
    attr_md.wr_auth           = 0;
    attr_md.vlen              = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_BMS_MAX_SETTINGS_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_bms->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_bms->settings_handles);
}

/**@brief       Function for adding characteristic.
 *
 * @param[in]   p_bms        Beacon Management Service structure.
 * @param[in]   p_bms_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t add_characteristic(ble_bms_t *p_bms,
                                   const ble_bms_init_t *p_bms_init,
                                   int char_uuid,
                                   uint16_t max_value_length,
                                   const ble_gatts_char_handles_t *p_handles)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write            = 1;
    char_md.char_props.write_wo_resp    = 1;
    char_md.p_char_user_desc            = NULL;
    char_md.p_char_pf                   = NULL;
    char_md.p_user_desc_md              = NULL;
    char_md.p_cccd_md                   = NULL;
    char_md.p_sccd_md                   = NULL;

    ble_uuid.type                       = p_bms->uuid_type;
    ble_uuid.uuid                       = char_uuid;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc                        = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth                     = 0;
    attr_md.wr_auth                     = 0;
    attr_md.vlen                        = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid              = &ble_uuid;
    attr_char_value.p_attr_md           = &attr_md;
    attr_char_value.init_len            = 1;
    attr_char_value.init_offs           = 0;
    attr_char_value.max_len             = max_value_length;

    return sd_ble_gatts_characteristic_add(p_bms->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           p_handles);
}


void ble_bms_on_ble_evt(ble_bms_t *p_bms, ble_evt_t *p_ble_evt)
{
    if ((p_bms == NULL) || (p_ble_evt == NULL))
    {
        return;
    }
/*
    uint16_t evt_id = p_ble_evt->header.evt_id;
    switch (p_ble_evt->header.evt_id)
    {
      case   BLE_GAP_EVT_CONNECTED:
        printf("BLE_GAP_EVT_CONNECTED = %d", evt_id); break;
      case   BLE_GAP_EVT_DISCONNECTED:
        printf("BLE_GAP_EVT_DISCONNECTED = %d", evt_id); break;
      case   BLE_GAP_EVT_CONN_PARAM_UPDATE:
        printf("BLE_GAP_EVT_CONN_PARAM_UPDATE = %d", evt_id); break;
      case   BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        printf("BLE_GAP_EVT_SEC_PARAMS_REQUEST = %d", evt_id); break;
      case   BLE_GAP_EVT_SEC_INFO_REQUEST:
        printf("BLE_GAP_EVT_SEC_INFO_REQUEST = %d", evt_id); break;
      case   BLE_GAP_EVT_PASSKEY_DISPLAY:
        printf("BLE_GAP_EVT_PASSKEY_DISPLAY = %d", evt_id); break;
      case   BLE_GAP_EVT_KEY_PRESSED:
        printf("BLE_GAP_EVT_KEY_PRESSED = %d", evt_id); break;
      case   BLE_GAP_EVT_AUTH_KEY_REQUEST:
        printf("BLE_GAP_EVT_AUTH_KEY_REQUEST = %d", evt_id); break;
      case   BLE_GAP_EVT_LESC_DHKEY_REQUEST:
        printf("BLE_GAP_EVT_LESC_DHKEY_REQUEST = %d", evt_id); break;
      case   BLE_GAP_EVT_AUTH_STATUS:
        printf("BLE_GAP_EVT_AUTH_STATUS = %d", evt_id); break;
      case   BLE_GAP_EVT_CONN_SEC_UPDATE:
        printf("BLE_GAP_EVT_CONN_SEC_UPDATE = %d", evt_id); break;
      case   BLE_GAP_EVT_TIMEOUT:
        printf("BLE_GAP_EVT_TIMEOUT = %d", evt_id); break;
      case   BLE_GAP_EVT_RSSI_CHANGED:
        printf("BLE_GAP_EVT_RSSI_CHANGED = %d", evt_id); break;
      case   BLE_GAP_EVT_ADV_REPORT:
        printf("BLE_GAP_EVT_ADV_REPORT = %d", evt_id); break;
      case   BLE_GAP_EVT_SEC_REQUEST:
        printf("BLE_GAP_EVT_SEC_REQUEST = %d", evt_id); break;
      case   BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        printf("BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST = %d", evt_id); break;
      case   BLE_GAP_EVT_SCAN_REQ_REPORT:
        printf("BLE_GAP_EVT_SCAN_REQ_REPORT = %d", evt_id); break;
      case   BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        printf("BLE_GAP_EVT_PHY_UPDATE_REQUEST = %d", evt_id); break;
      case   BLE_GAP_EVT_PHY_UPDATE:
        printf("BLE_GAP_EVT_PHY_UPDATE = %d", evt_id); break;
      case   BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        printf("BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST = %d", evt_id); break;
      case   BLE_GAP_EVT_DATA_LENGTH_UPDATE:
        printf("BLE_GAP_EVT_DATA_LENGTH_UPDATE = %d", evt_id); break;
      case   BLE_GAP_EVT_QOS_CHANNEL_SURVEY_REPORT:
        printf("BLE_GAP_EVT_QOS_CHANNEL_SURVEY_REPORT = %d", evt_id); break;
      case   BLE_GAP_EVT_ADV_SET_TERMINATED: 
        printf("BLE_GAP_EVT_ADV_SET_TERMINATED  = %d", evt_id); break;
      default:
        printf("Event Not Found  = %d", evt_id); break;
    }
*/
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            //printf("on_connect");
            on_connect(p_bms, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            //printf("on_disconnect");
            on_disconnect(p_bms, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            //printf("on_write");
            on_write(p_bms, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

uint32_t ble_bms_init(ble_bms_t *p_bms)
{
  
    uint32_t        err_code;
    //pstorage_module_param_t param;
    ble_uuid_t      ble_uuid;
    ble_bms_init_t   bms_init;
    ble_bms_init_t   *p_bms_init = &bms_init;

    memset(&bms_init, 0, sizeof(bms_init));
    bms_init.data_handler = bms_data_handler;

    //if ((p_bms == NULL) || (p_bms_init == NULL))
    if (p_bms == NULL) {
        return NRF_ERROR_NULL;
    }

    //--------------------------------------------------------------------------
    // Initialize service structure.
    p_bms->conn_handle              = BLE_CONN_HANDLE_INVALID;
    p_bms->data_handler             = p_bms_init->data_handler;
    p_bms->is_notification_enabled  = false;

    ble_uuid128_t bms_base_uuid = {BMS_UUID_BASE}; // -Wall

    /**@snippet [Adding proprietary Service to S110 SoftDevice] */

    // Add custom base UUID.
    err_code = sd_ble_uuid_vs_add(&bms_base_uuid, &p_bms->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    ble_uuid.type = p_bms->uuid_type;
    ble_uuid.uuid = BLE_UUID_BMS_SERVICE_;

    // Add service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_bms->service_handle);
    /**@snippet [Adding proprietary Service to S110 SoftDevice] */
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    //--------------------------------------------------------------------------
    // Add Settings Characteristic.
    err_code = settings_char_add(p_bms, p_bms_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Entering DFU Mode Characteristic.
    err_code = add_characteristic(p_bms, p_bms_init,
                                  BLE_UUID_BMS_DFU_CHARACTERISTIC, 3,
                                  &p_bms->dfu_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add LED Control Characteristic.
    err_code = add_characteristic(p_bms, p_bms_init,
                                  BLE_UUID_BMS_LED_CHARACTERISTIC, 5,
                                  &p_bms->led_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Factory Check Mode Characteristic.
    err_code = add_characteristic(p_bms, p_bms_init,
                                  BLE_UUID_BMS_FCM_CHARACTERISTIC, 5,
                                  &p_bms->fcm_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Battery Calibration Mode Characteristic.
    err_code = add_characteristic(p_bms, p_bms_init,
                                  BLE_UUID_BMS_BCM_CHARACTERISTIC, 5,
                                  &p_bms->bcm_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Reset Hardware Characteristic.
    err_code = add_characteristic(p_bms, p_bms_init,
                                  BLE_UUID_BMS_RST_CHARACTERISTIC, 5,
                                  &p_bms->rst_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Reset Hardware Characteristic.
    err_code = add_characteristic(p_bms, p_bms_init,
                                  BLE_UUID_BMS_PWR_CHARACTERISTIC, 5,
                                  &p_bms->pwr_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Factory Default (Flash Clear & Hardware Reset) Characteristic.
    err_code = add_characteristic(p_bms, p_bms_init,
                                  BLE_UUID_BMS_FDM_CHARACTERISTIC, 5,
                                  &p_bms->fdm_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

#define OP_SEC_LEN 4 // opcode(2byte) + key(1byte) + match(1byte)

static void bms_data_handler(ble_bms_t *p_bms, uint8_t *p_data, uint16_t length, uint16_t handle)
{
  uint32_t         err_code;

  uint8_t *_beacon_info = ble_bms_get_beacon_info();

  // Access Control
  if (handle == p_bms->settings_handles.value_handle
      || handle == p_bms->dfu_handles.value_handle) {
    uint8_t ac_key    = p_data[length - 2];
    uint8_t ac_match  = p_data[length - 1];
    if (ac_verify(ac_key,ac_match) == false) {
      APP_ERROR_CHECK(NRF_ERROR_FORBIDDEN);
    }
  }

  if (handle == p_bms->settings_handles.value_handle) {

    int i=0;
    
    if (length < 3) return;

    // 00F0 : UUID
    if (p_data[i] == 0x00 && p_data[i+1] == 0xF0) {
      int idx = BINFO_IBEACON_UUID_IDX;
      bool m_has_changed = false;
      for (int j=(i+2); j < (i+2)+16; j++) {
        if (_beacon_info[idx] != p_data[j]) m_has_changed = true;
        _beacon_info[idx] = p_data[j];
        idx++;
      }

      if (m_has_changed) {
        if (_beacon_info[BINFO_STATUS_VALUE_IDX] == 0 || 
            _beacon_info[BINFO_STATUS_VALUE_IDX] == 2 || 
            _beacon_info[BINFO_STATUS_VALUE_IDX] == 0xFF) {
          // 未登録でUUID/Major/Minorに変更があった場合、ステータスを初期設定済みにする
          _beacon_info[BINFO_STATUS_VALUE_IDX] = 2;
        } 
        else {
          // 未登録以外でUUID/Major/Minorに変更があった場合、ステータスを再初期設定済みにする
          _beacon_info[BINFO_STATUS_VALUE_IDX] = 3;
        }
      }

    }

    // 00F1 : Major
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xF1) {
      int idx = BINFO_MAJOR_VALUE_IDX;
      bool m_has_changed = false;
      for (int j=(i+2); j < (i+2)+2; j++) {
        if (_beacon_info[idx] != p_data[j]) m_has_changed = true;
        _beacon_info[idx] = p_data[j];
        idx++;
      }
      
      if (m_has_changed) {
        if (_beacon_info[BINFO_STATUS_VALUE_IDX] == 0 || 
            _beacon_info[BINFO_STATUS_VALUE_IDX] == 2 || 
            _beacon_info[BINFO_STATUS_VALUE_IDX] == 0xFF) {
          // 未登録でUUID/Major/Minorに変更があった場合、ステータスを初期設定済みにする
          _beacon_info[BINFO_STATUS_VALUE_IDX] = 2;
        }
        else {
          // 未登録以外でUUID/Major/Minorに変更があった場合、ステータスを再初期設定済みにする
          _beacon_info[BINFO_STATUS_VALUE_IDX] = 3;
        }
      }

    }

    // 00F2 : Minor
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xF2) {
      int idx = BINFO_MINOR_VALUE_IDX;
      bool m_has_changed = false;
      for (int j=(i+2); j < (i+2)+2; j++) {
        if (_beacon_info[idx] != p_data[j]) m_has_changed = true;
        _beacon_info[idx] = p_data[j];
        idx++;
      }

      if (m_has_changed) {
        if (_beacon_info[BINFO_STATUS_VALUE_IDX] == 0 || 
            _beacon_info[BINFO_STATUS_VALUE_IDX] == 2 || 
            _beacon_info[BINFO_STATUS_VALUE_IDX] == 0xFF) {
          // 未登録でUUID/Major/Minorに変更があった場合、ステータスを初期設定済みにする
          _beacon_info[BINFO_STATUS_VALUE_IDX] = 2;
        }
        else {
          // 未登録以外でUUID/Major/Minorに変更があった場合、ステータスを再初期設定済みにする
          _beacon_info[BINFO_STATUS_VALUE_IDX] = 3;
        }
      }

    }

    // 00F3 : Status
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xF3) {
      uint8_t status = p_data[i+2];

#if 0
      if (status == 9) {
        m_ibeacon_mode = 1;
        status = 4;
      }
#endif /* if 0 */

      _beacon_info[BINFO_STATUS_VALUE_IDX] = status;

#if defined (LED_ENABLED)
      if (_beacon_info[BINFO_STATUS_VALUE_IDX] == 0x00) {
        execute_led(LED_ON);
      }
      else {
        execute_led(LED_OFF);
      }
#endif

    }

    // 00F4 : Tx Power
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xF4) {
      _beacon_info[BINFO_TXPWR_VALUE_IDX] = p_data[i+2];
      int8_t txPowerLevel = get_tx_power_level( _beacon_info[BINFO_TXPWR_VALUE_IDX] );
      err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, txPowerLevel);
      APP_ERROR_CHECK(err_code);
      _beacon_info[BINFO_RSSI_VALUE_IDX] = 0x00; // reset RSSI to 0
      //print("00F4 : Tx Power = %d", _beacon_info[BINFO_TXPWR_VALUE_IDX] );
      //print("00F4 : Tx Power");
    }

    // 00F5 : Tx Frequency
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xF5) {
      _beacon_info[BINFO_TXFRQ_VALUE_IDX] = p_data[i+2];
    }

    // 00F6 : Battery
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xF6) {
      _beacon_info[BINFO_BATTERY_VALUE_IDX] = p_data[i+2];
    }

    // 00F7 : Serial ID
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xF7) {
      if (length != BINFO_SERIAL_ID_VALUE_SIZ+OP_SEC_LEN) return;

      _beacon_info[BINFO_SERIAL_ID_VALUE_IDX] = p_data[i+2];
      _beacon_info[BINFO_SERIAL_ID_VALUE_IDX+1] = p_data[i+3];
      _beacon_info[BINFO_SERIAL_ID_VALUE_IDX+2] = p_data[i+4];
      _beacon_info[BINFO_SERIAL_ID_VALUE_IDX+3] = p_data[i+5];
    }

    // 00F8 : Service ID
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xF8) {
      if (length != BINFO_SERVICE_ID_VALUE_SIZ+OP_SEC_LEN) return;

      _beacon_info[BINFO_SERVICE_ID_VALUE_IDX] = p_data[i+2];
      _beacon_info[BINFO_SERVICE_ID_VALUE_IDX+1] = p_data[i+3];
    }

    // 00F9 : Geo Hash
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xF9) {
      if (length != BINFO_GEO_HASH_VALUE_SIZ+OP_SEC_LEN) return;

      int idx = BINFO_GEO_HASH_VALUE_IDX;
      for (int j=(i+2); j < (i+2) + 10; j++) {
        _beacon_info[idx++] = p_data[j];
      }
    }

    // 00FA : Firmware version
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xFA) {
      if (length != BINFO_VERSION_VALUE_SIZ+OP_SEC_LEN) return;

      _beacon_info[BINFO_VERSION_VALUE_IDX] = p_data[i+2];
      _beacon_info[BINFO_VERSION_VALUE_IDX+1] = p_data[i+3];
    }

    // 00FB : RSSI    
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xFB) {
      _beacon_info[BINFO_RSSI_VALUE_IDX] = p_data[i+2];
    }

    // 00FF : commit
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xFF) {
      if(p_data[i+2] == 0x01) {
        //NRF_LOG_INFO("tb_manager_settings_store");
        err_code = tb_manager_settings_store();
        APP_ERROR_CHECK(err_code);
        //led_param = 1;
      }
      else if(p_data[i+2] == 0x02) { // reset beacon_info
        ble_bms_reset_beacon_info();
      }
      
    }

    // 00E0: EddyStone-URL #1
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xE0) {  
      if (length < 4) return;

      uint8_t flag = p_data[i+2];
      uint8_t req_btc = (flag & 0b10000000) >> 7;
      uint8_t req_sch = (flag & 0b01100000) >> 5;
      uint8_t req_len = (flag & 0b00011111);

      int len = 0;
      if (req_btc == 0x01) {
        // 3 characters on tail of URL are used to represent battery status.
        // So, max len of input string is 14 characters.
        len = req_len <= 14 ? req_len : 14;
      }
      else {
        len = req_len <= 17 ? req_len : 17;
      }
          
      for (int j = 0, k = i+3; j < len; ++j, ++k) {
        _beacon_info[BINFO_EDDYSTONE1_URL_URL_IDX+j] = p_data[k];
      }

      _beacon_info[BINFO_EDDYSTONE1_URL_LEN_IDX] = req_len;
      _beacon_info[BINFO_EDDYSTONE1_URL_SCH_IDX] = req_sch;
      _beacon_info[BINFO_EDDYSTONE1_URL_BTC_IDX] = req_btc;

    }

    // 00E1 : Advertising setting
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xE1) {
      _beacon_info[ADVERTISE_SETTING_IDX] = p_data[i+2];
    }

    // 00E2: EddyStone-URL #2
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xE2) {  
      if (length < 4) return;

      uint8_t flag = p_data[i+2];
      uint8_t req_btc = (flag & 0b10000000) >> 7;
      uint8_t req_sch = (flag & 0b01100000) >> 5;
      uint8_t req_len = (flag & 0b00011111);

      int len = 0;
      if (req_btc == 0x01) {
        // 3 characters on tail of URL are used to represent battery status.
        // So, max len of input string is 14 characters.
        len = req_len <= 14 ? req_len : 14;
      }
      else {
        len = req_len <= 17 ? req_len : 17;
      }
          
      for (int j = 0, k = i+3; j < len; ++j, ++k) {
        _beacon_info[BINFO_EDDYSTONE2_URL_URL_IDX+j] = p_data[k];
      }

      _beacon_info[BINFO_EDDYSTONE2_URL_LEN_IDX] = req_len;
      _beacon_info[BINFO_EDDYSTONE2_URL_SCH_IDX] = req_sch;
      _beacon_info[BINFO_EDDYSTONE_URL_BTC_IDX] = req_btc;

    }

    // 00E3: set mode list
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xE3) {
      if (length != BINFO_MODE_LIST_SIZ+OP_SEC_LEN) return;

      for (int j = 0, k = i+2; j < BINFO_MODE_LIST_SIZ; ++j, ++k) {
        _beacon_info[BINFO_MODE_LIST_IDX+j] = p_data[k];
      }
    }

    // 00E4: set TxPower for management
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xE4) {
      _beacon_info[BINFO_TXPWR_FOR_MNG_IDX] = p_data[i+2];
      int8_t txPowerLevel = get_tx_power_level( _beacon_info[BINFO_TXPWR_FOR_MNG_IDX] );
      err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, get_sd_adv_handle(), txPowerLevel);
      APP_ERROR_CHECK(err_code);
    }

    // 00E5: EddyStone-UID ID
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xE5) {  
      if (length != BINFO_EDDYSTONE_UID_ID_SIZ+OP_SEC_LEN) return;
      
      for (int j = 0; j < BINFO_EDDYSTONE_UID_ID_SIZ; ++j) {
        _beacon_info[BINFO_EDDYSTONE_UID_ID_IDX+j] = p_data[j+2];
      }

    }

    // 00E6: EddyStone-UID Namespace
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xE6) {  
      if (length != BINFO_EDDYSTONE_UID_NAMESPACE_SIZ+OP_SEC_LEN) return;
          
      for (int j = 0; j < BINFO_EDDYSTONE_UID_NAMESPACE_SIZ; ++j) {
        _beacon_info[BINFO_EDDYSTONE_UID_NAMESPACE_IDX+j] = p_data[j+2];
      }

    }

    // 00E7: LINE Beacon HWID
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xE7) {  
      if (length != BINFO_LINE_BEACON_HWID_SIZ+OP_SEC_LEN) return;
          
      for (int j = 0; j < BINFO_LINE_BEACON_HWID_SIZ; ++j) {
        _beacon_info[BINFO_LINE_BEACON_HWID_IDX+j] = p_data[j+2];
      }

    }

    // 00E8: LINE Beacon VendorKey
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xE8) {  
      if (length != BINFO_LINE_BEACON_VENDOR_KEY_SIZ+OP_SEC_LEN) return;
          
      for (int j = 0; j < BINFO_LINE_BEACON_VENDOR_KEY_SIZ; ++j) {
        _beacon_info[BINFO_LINE_BEACON_VENDOR_KEY_IDX+j] = p_data[j+2];
      }

    }

    // 00E9: LINE Beacon LotKey
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xE9) {  
      if (length != BINFO_LINE_BEACON_LOTKEY_SIZ+OP_SEC_LEN) return;
          
      for (int j = 0; j < BINFO_LINE_BEACON_LOTKEY_SIZ; ++j) {
        _beacon_info[BINFO_LINE_BEACON_LOTKEY_IDX+j] = p_data[j+2];
      }

    }

    // 00EA : SYSCONFIG FLAGS
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xEA) {
      _beacon_info[BINFO_SYSCONFIG_FLAGS_IDX]= p_data[i+2];
    }

    // 00EB : RESET 15sec timestamp counter
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xEB) {
      if (length != BINFO_15SEC_TIMESTAMP_SIZ+OP_SEC_LEN) return;

      _beacon_info[BINFO_15SEC_TIMESTAMP_IDX]= p_data[i+2];
      _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+1]= p_data[i+3];
      _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+2]= p_data[i+4];
      _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+3]= p_data[i+5];
      _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+4]= p_data[i+6];
      _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+5]= p_data[i+7];
      _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+6]= p_data[i+8];
      _beacon_info[BINFO_15SEC_TIMESTAMP_IDX+7]= p_data[i+9];

      restore_15sec_timestamp();
    }

    // 00EC : Tangerine Secure iBeacon Secure Key
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xEC) {
      if (length != BINFO_TGSECB_SEC_KEY_SIZ+OP_SEC_LEN) return;

      _beacon_info[BINFO_TGSECB_SEC_KEY_IDX]   = p_data[i+2];
      _beacon_info[BINFO_TGSECB_SEC_KEY_IDX+1] = p_data[i+3];
      _beacon_info[BINFO_TGSECB_SEC_KEY_IDX+2] = p_data[i+4];
      _beacon_info[BINFO_TGSECB_SEC_KEY_IDX+3] = p_data[i+5];
      _beacon_info[BINFO_TGSECB_SEC_KEY_IDX+4] = p_data[i+6];
      _beacon_info[BINFO_TGSECB_SEC_KEY_IDX+5] = p_data[i+7];      
    }

    // 00ED : Rest Tangerine Secure iBeacon Timestamp
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xED) {
      if (length != BINFO_TGSECB_TIMESTAMP_SIZ+OP_SEC_LEN) return;

      _beacon_info[BINFO_TGSECB_TIMESTAMP_IDX]   = p_data[i+2];
      _beacon_info[BINFO_TGSECB_TIMESTAMP_IDX+1] = p_data[i+3];
      _beacon_info[BINFO_TGSECB_TIMESTAMP_IDX+2] = p_data[i+4];
      _beacon_info[BINFO_TGSECB_TIMESTAMP_IDX+3] = p_data[i+5];

      restore_15sec_tgsec_timestamp();
    }

    // 00EF : SET Current datetime
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xEF) {
      if (length != BINFO_CURRENT_DATETIME_SIZ+OP_SEC_LEN) return;

      _beacon_info[BINFO_SET_CURRENT_DATETIME_IDX]= p_data[i+2];
      _beacon_info[BINFO_SET_CURRENT_DATETIME_IDX+1]= p_data[i+3];
      _beacon_info[BINFO_SET_CURRENT_DATETIME_IDX+2]= p_data[i+4];
      _beacon_info[BINFO_SET_CURRENT_DATETIME_IDX+3]= p_data[i+5];
      _beacon_info[BINFO_SET_CURRENT_DATETIME_IDX+4]= p_data[i+6];
      _beacon_info[BINFO_SET_CURRENT_DATETIME_IDX+5]= p_data[i+7];
      _beacon_info[BINFO_SET_CURRENT_DATETIME_IDX+6]= p_data[i+8];
      memcpy(&m_pre_time, &_beacon_info[BINFO_SET_CURRENT_DATETIME_IDX], BINFO_CURRENT_DATETIME_SIZ);

      m_system_timer = (bcd2bin(m_pre_time.hours) * 3600) + (bcd2bin(m_pre_time.minutes) * 60)  + bcd2bin(m_pre_time.seconds);
  #ifdef  DEBUG_RTC_ENABLE
      NRF_LOG_INFO("00EF : SET Current datetime %02d:%02d:%02d -> %d", bcd2bin(m_pre_time.hours), bcd2bin(m_pre_time.minutes), bcd2bin(m_pre_time.seconds), m_system_timer);
  #endif
      pcf8563_write();
    }

    // 00D1 : SET ECO Mode Start Time
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xD0) {
      if (length != BINFO_ECO_MODE_START_TIME_SIZ+OP_SEC_LEN) return;

      _beacon_info[BINFO_ECO_MODE_START_TIME_IDX]   = p_data[i+2];
      _beacon_info[BINFO_ECO_MODE_START_TIME_IDX+1] = p_data[i+3];
    }

    // 00D2 : SET ECO Mode Finish Time
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xD1) {
      if (length != BINFO_ECO_MODE_FINISH_TIME_SIZ+OP_SEC_LEN) return;

      _beacon_info[BINFO_ECO_MODE_FINISH_TIME_IDX]   = p_data[i+2];
      _beacon_info[BINFO_ECO_MODE_FINISH_TIME_IDX+1] = p_data[i+3];
    }

    // 00D0 : SET Enable/Disable TIMESLOT mode & status
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xD2) {
      uint8_t before_mode = _beacon_info[BINFO_TIMESLOT_MODE_STATUS_IDX];
      _beacon_info[BINFO_TIMESLOT_MODE_STATUS_IDX]= p_data[i+2] & 0x8F;
      if (before_mode != _beacon_info[BINFO_TIMESLOT_MODE_STATUS_IDX]) m_bPending = true;
      //NRF_LOG_INFO("BINFO_TIMESLOT_MODE_STATUS_IDX = %02X", _beacon_info[BINFO_TIMESLOT_MODE_STATUS_IDX]);

      //set_timeslot_mode();
    }

    // 00D3: Set Radio Timeslot length list
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xD3) {
      _beacon_info[BINFO_TIMESLOT_TXFRQ_VALUE_IDX] = p_data[i+2];
    }

    // 00D4: Set Radio Timeslot advertising distance list
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xD4) {
      _beacon_info[BINFO_TBM_TXFRQ_VALUE_IDX]   = p_data[i+2];
    }

    // 00D5: Set Battery Max Capacity Value
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xD5) {
      if (length != BINFO_BMS_BATTERY_MAX_CAPACITY_SIZ+OP_SEC_LEN) return;

      _beacon_info[BINFO_BMS_BATTERY_MAX_CAPACITY_IDX]   = p_data[i+3];
      _beacon_info[BINFO_BMS_BATTERY_MAX_CAPACITY_IDX+1] = p_data[i+2];
      uint16_t *pBattery_Voltage_Max_Capacity = (uint16_t *)&_beacon_info[BINFO_BMS_BATTERY_MAX_CAPACITY_IDX]; 
      m_Battery_Voltage_Max_Capacity = (uint16_t)*pBattery_Voltage_Max_Capacity;
    }

    // 00D6: Set Hardware Type
    else if (p_data[i] == 0x00 && p_data[i+1] == 0xD6) {
      _beacon_info[BINFO_HARDWARE_TYPE_IDX]   = p_data[i+2];

      if ( _beacon_info[BINFO_HARDWARE_TYPE_IDX] == HW_TYPE_TANGERINE_BEACON ) {
        uint8_t max_battery_valtage[2] = {ENERGIZER_MAXIMUM_CAPACITY};
        _beacon_info[BINFO_BMS_BATTERY_MAX_CAPACITY_IDX]   = max_battery_valtage[1];
        _beacon_info[BINFO_BMS_BATTERY_MAX_CAPACITY_IDX+1] = max_battery_valtage[0];
      }
      else {
        uint8_t max_battery_valtage[2] = {MAXBEACON_MAXIMUM_CAPACITY};
        _beacon_info[BINFO_BMS_BATTERY_MAX_CAPACITY_IDX]   = max_battery_valtage[1];
        _beacon_info[BINFO_BMS_BATTERY_MAX_CAPACITY_IDX+1] = max_battery_valtage[0];
      }
    }

    // rebuild packet data
    build_all_data();

  }

  // Entering DFU mode
  else if (handle == p_bms->dfu_handles.value_handle) {
    if ( m_hardware_type == HW_TYPE_MINEW_MAX_BEACON ) nrf_gpio_cfg_sense_set(SW_HW_MIMAXK, NRF_GPIO_PIN_SENSE_LOW);
    bootloader_start();
  }

  // Flash LED
  else if (handle == p_bms->led_handles.value_handle) {

#if defined(LED_ENABLED)
    m_Execute_led_flash_type1 = true;
#endif

  }
    
  // Factory Check mode
  else if (handle == p_bms->fcm_handles.value_handle) {
    _beacon_info[BINFO_MODE_LIST_IDX] = BLE_ADV_MODE_BMS;
    _beacon_info[BINFO_STATUS_VALUE_IDX] = 4; // アクティブ
    g_startup_stage = 1;
    m_fcm = 0xFF;
  }

  // Battery Calibration mode
  else if (handle == p_bms->bcm_handles.value_handle) {
    uint16_t blevel = 0;
    uint8_t bBatPsent = 0;

    for(int ix=0; ix < 10; ix++) {
      blevel = get_battery_level();
      bBatPsent = battery_level_to_percent(blevel);
      if (bBatPsent > 10) break;
    }
    float flevel = (float)blevel - ((float)blevel * 0.01f);
    blevel = flevel; 

    _beacon_info[BINFO_BMS_BATTERY_MAX_CAPACITY_IDX+1]   = (uint8_t)((blevel & 0xFF00) >> 8);
    _beacon_info[BINFO_BMS_BATTERY_MAX_CAPACITY_IDX] = (uint8_t)(blevel & 0x00FF);
    uint16_t *pBattery_Voltage_Max_Capacity = (uint16_t *)&_beacon_info[BINFO_BMS_BATTERY_MAX_CAPACITY_IDX]; 
    m_Battery_Voltage_Max_Capacity = (uint16_t)*pBattery_Voltage_Max_Capacity;
  }

  // Reset Hardware
  else if (handle == p_bms->rst_handles.value_handle) {
    NVIC_SystemReset();
  }

  // Power OFF
  else if (handle == p_bms->pwr_handles.value_handle) {
    blink_pending_led(3, 200);
    nrf_delay_ms(2000);
    if ( m_hardware_type == HW_TYPE_MINEW_MAX_BEACON ) nrf_gpio_cfg_sense_set(SW_HW_MIMAXK, NRF_GPIO_PIN_SENSE_LOW);
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
  }

  // Factory Default (Flash Clear & Hardware Reset) Characteristic.
  else if (handle == p_bms->fdm_handles.value_handle) {
    execute_led(LED_ON);
    if ( m_hardware_type == HW_TYPE_MINEW_MAX_BEACON ) nrf_gpio_cfg_sense_set(SW_HW_MIMAXK, NRF_GPIO_PIN_SENSE_LOW);
    err_code = tb_manager_reset();
    APP_ERROR_CHECK(err_code);
    nrf_delay_ms(3000);
    execute_led(LED_OFF);
    NVIC_SystemReset();
  }


}

static uint8_t ac_last_key;
static bool ac_verify(uint8_t ac_key, uint8_t ac_match) {
  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  if (ac_key == ac_last_key) {
    // 同じキーは使えない
    return false;
  }
  uint8_t ac_salt   = (_beacon_info[40] + _beacon_info[41]);
  uint8_t ac_calc_result = (ac_salt + ac_key) * 2;
  bool ok = (ac_calc_result == ac_match);
  if (ok) {
    ac_last_key = ac_key;
  }
  return ok;
}

uint8_t get_status_flags()
{
  uint8_t *_beacon_info = ble_bms_get_beacon_info();

  // Battery Supply / Beacon Battery / Beacon Status
  uint8_t statusFlags = _beacon_info[BINFO_BATTERY_VALUE_IDX];

  // Beacon Status
  /* 
     0000:Not Provisioned
     0001:Provisioned
     0010:Initialized
     0011:Reinitialized
     0100:Active
     0101:Inactive
     0110:Removed
     0111:Terminated
  */
  switch( _beacon_info[BINFO_STATUS_VALUE_IDX] ) {
  case 0: // TGRBeaconStatusNotProvisioned, // 未登録
    break;
  case 1: // TGRBeaconStatusProvisioned, // 登録済み
    statusFlags |= 1;
    break;
  case 2: // TGRBeaconStatusInitialized, // 初期設定済み
    statusFlags |= (1 << 1);
    break;
  case 3: // TGRBeaconStatusReinitialized, // 再初期設定済み
    statusFlags |= 1;
    statusFlags |= (1 << 1);
    break;
  case 4: // TGRBeaconStatusActive, // アクティブ
    statusFlags |= (1 << 2);
    break;
  case 5: // TGRBeaconStatusInactive, // 休止中
    statusFlags |= 1;
    statusFlags |= (1 << 2);
    break;
  case 6: // TGRBeaconStatusRemoved, // 撤去済み
    statusFlags |= (1 << 1);
    statusFlags |= (1 << 2);
    break;
  case 7: // TGRBeaconStatusTerminated, // 終了
    statusFlags |= 1;
    statusFlags |= (1 << 1);
    statusFlags |= (1 << 2);
    break;
  default: // TGRBeaconStatusUnknown
    statusFlags |= (1 << 3);
    break;
  }
 
  return statusFlags;
}

void set_timeslot_mode(void)
{
  uint8_t *_beacon_info = ble_bms_get_beacon_info();

  m_timeslot_mode = _beacon_info[BINFO_TIMESLOT_MODE_STATUS_IDX];
  if (m_timeslot_mode != 0x00) {
    switch(m_timeslot_mode & 0x0f) {
    case 0 :  // Factory Default
      m_advertising_packet_type = 0x40; break;
    case 1 :  // iBeacon
      m_advertising_packet_type = 0x01; break;
    case 2 :  // Secure iBeacon
      m_advertising_packet_type = 0x20; break;
    case 3 :  // LINE
      m_advertising_packet_type = 0x10; break;
    case 4 :  // LINE + iBeacon
      m_advertising_packet_type = 0x11; break;
    case 5 :  // LINE + Secure iBeacon
      m_advertising_packet_type = 0x30; break;
    case 6 :  // flxBeacon
      m_advertising_packet_type = 0x40; break;
    case 7 :  // iBeacon / flxBeacon
      m_advertising_packet_type = 0x41; break;
    case 8 :  // Secure iBeacon / flxBeacon
      m_advertising_packet_type = 0x60; break;
    case 9 :  // LINE + Secure iBeacon / flxBeacon
      m_advertising_packet_type = 0x51; break;
    case 10 : // LINE + Secure iBeacon / flxBeacon
      m_advertising_packet_type = 0x70; break;
    default :
      m_advertising_packet_type = 0x40; break;
    }  
  }
}
