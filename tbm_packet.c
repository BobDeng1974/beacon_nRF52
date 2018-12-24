#include "tbm_packet.h"


static uint8_t bms_info[APP_BMS_INFO_LENGTH] =                  /**< Information advertised by BMS. */
{
    0x00, 0x00,                                                  // Service ID : 2 bytes
    0x00, 0x00, 0x00, 0x00,                                      // Serial ID : 4 bytes
    0x00, 0x00, 0x00, 0x00,                                      // Beacon ID : 4 bytes
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // Geo Hash : 10 bytes
    0x00,                                                        // Battery/Status 1 byte
    0x00,                                                        // Tx Power/Frequency 1 byte
    APP_FIRMWARE_VERSION_VALUE                                   // 2 bytes
}; // 24bytes

static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scrdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];    /**< Buffer for storing an encoded scan response data set. */

static uint8_t prev_status = 1;

uint8_t m_fcm = 0;

/**@brief Building Tangerine Beacon Management advertising data
*/
void build_bms_data(void)
{
  //
  // Preset & Update Beacon settings
  //
  uint8_t *_beacon_info = ble_bms_get_beacon_info();

  // Service ID: 0-1
  // 28-29
  for (int i = BINFO_SERVICE_ID_VALUE_IDX; i < (BINFO_SERVICE_ID_VALUE_IDX+2); i++) {
    bms_info[i-28] = _beacon_info[i];
  }

  // Serial ID: 2-5
  // 140-143 
  if (g_startup_stage == 1) {
    // order: mem: little endian, packet: big endian
    for (int i = BINFO_TGSECB_TIMESTAMP_IDX; i < (BINFO_TGSECB_TIMESTAMP_IDX+4); i++) {
      bms_info[i-138] = _beacon_info[283-i]; // [i-138] <= [143-i+143]
    }
  }
  else {
    // 24-27 
    for (int i = BINFO_SERIAL_ID_VALUE_IDX; i < (BINFO_SERIAL_ID_VALUE_IDX+4); i++) {
      bms_info[i-22] = _beacon_info[i];
    }
  }

  // Beacon ID (Major/Minor)
  for (int i = BINFO_MAJOR_VALUE_IDX; i < (BINFO_MAJOR_VALUE_IDX+4); i++) {
    bms_info[i-10] = _beacon_info[i];
    //print("%d - ", _beacon_info[i]);
  }
   
  // GEO Hash
  for (int i = BINFO_GEO_HASH_VALUE_IDX; i < (BINFO_GEO_HASH_VALUE_IDX+BINFO_GEO_HASH_VALUE_SIZ); i++) {
    bms_info[i-20] = _beacon_info[i];
  }

  // Factory Mode
  if ( m_fcm == 0xFF ) {
    if (pcf8563_read()) {
      memcpy(&bms_info[10], &m_pre_time, sizeof(m_pre_time));
    }
    else {
     memset(&bms_info[10], 0xFF, sizeof(m_pre_time));
    }

    bms_info[17] = battery_level_get2();
    bms_info[18] = battery_level_get2();
  }

  // Battery Supply / Beacon Battery / Beacon Status
  uint8_t statusFlags = _beacon_info[BINFO_BATTERY_VALUE_IDX];

  // Beacon Status
  /* 
     000:Not Provisioned
     001:Provisioned
     010:Initialized
     011:Reinitialized
     100:Active
     101:Inactive
     110:Removed
     111:Terminated
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
    break;
  }

  // ECO mode status
  //statusFlags |= (_beacon_info[BINFO_ECO_MODE_STATUS_IDX] << 3);
  m_eco_start_time.dec.Hours     = _beacon_info[BINFO_ECO_MODE_START_TIME_IDX];
  m_eco_start_time.dec.Minutes   = _beacon_info[BINFO_ECO_MODE_START_TIME_IDX+1];
  m_eco_finish_time.dec.Hours    = _beacon_info[BINFO_ECO_MODE_FINISH_TIME_IDX];
  m_eco_finish_time.dec.Minutes  = _beacon_info[BINFO_ECO_MODE_FINISH_TIME_IDX+1];
  if (m_eco_start_time.wTime != m_eco_finish_time.wTime) statusFlags |= 0x08;

  bms_info[20] = statusFlags;
  if ( m_fcm == 0xFF ) bms_info[20] = m_fcm;

  // Tx Power / Beacon Frequency
  uint8_t txFlags = 0b00000000;
  
  // Tx Power
  /* 
     0000: -40dBm
     0001: -30dBm
     0010: -20dBm
     0011: -16dBm
     0100: -12dBm
     0101: -8dBm
     0110: -4dBm
     0111: 0dBm
     1000: 4dBm
   */
  switch( _beacon_info[BINFO_TXPWR_VALUE_IDX] ) {
  case 0:
    txFlags = (0 << 8);
    break;
  case 1:
    txFlags |= (1 << 4);
    break;
  case 2:
    txFlags |= (1 << 5);
    break;
  case 3:
    txFlags |= (1 << 4);
    txFlags |= (1 << 5);
    break;
  case 4:
    txFlags |= (1 << 6);
    break;
  case 5:
    txFlags |= (1 << 4);
    txFlags |= (1 << 6);
    break;
  case 6:
    txFlags |= (1 << 5);
    txFlags |= (1 << 6);
    break;
  case 7:
    txFlags |= (1 << 4);
    txFlags |= (1 << 5);
    txFlags |= (1 << 6);
    break;
  case 8:
    txFlags |= (1 << 7);
    break;
  }

  // Beacon Frequency
  switch( _beacon_info[BINFO_TXFRQ_VALUE_IDX] ) {
  case 0:
    txFlags |= 0b00000000;
    break;
  case 1:
    txFlags |= 0b00000001;
    break;
  case 2:
    txFlags |= 0b00000010;
    //txFlags |= (1 << 1);
    break;
  case 3:
    txFlags |= 0b00000011;
    //txFlags |= (1 << 0);
    //txFlags |= (1 << 1);
    break;
  case 4:
    txFlags |= 0b00000100;
    //txFlags |= (1 << 2);
    break;
  case 5:
    txFlags |= 0b00000101;
    //txFlags |= (1 << 0);
    //txFlags |= (1 << 2);
    break;
  case 6:
    txFlags |= 0b00000110;
    //txFlags |= (1 << 1);
    //txFlags |= (1 << 2);
    break;
  case 7:
    txFlags |= 0b00000111;
    //txFlags |= (1 << 0);
    //txFlags |= (1 << 1);
    //txFlags |= (1 << 2);
    break;

  case 8:
    txFlags |= 0b00001000;
    break;
  case 9:
    txFlags |= 0b00001001;
    break;
  case 10:
    txFlags |= 0b00001010;
    break;
  case 11:
    txFlags |= 0b00001011;
    break;
  case 12:
    txFlags |= 0b00001100;
    break;
  case 13:
    txFlags |= 0b00001101;
    break;
  case 14:
    txFlags |= 0b00001110;
    break;
  case 15:
    txFlags |= 0b00001111;
    break;
  }

  bms_info[21] = txFlags;

  // Firmware Version
  bms_info[22] = _beacon_info[BINFO_VERSION_VALUE_IDX];
  bms_info[23] = _beacon_info[BINFO_VERSION_VALUE_IDX+1];
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;

        case BLE_ADV_EVT_IDLE:
            //sleep_mode_enter();
            break;

        default:
            break;
    }
}

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
        .p_data = m_enc_scrdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    }
};

/**@brief Tangerine Beacon Management Service advertising initialization
 */
void bms_advertising_init(ble_bms_t m_bms)
{
  ret_code_t    err_code;
  ble_advdata_t advdata;
  ble_advdata_t scanrsp;
  uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

  //
  // UUIDs for services
  //
  ble_uuid_t adv_uuids[] = {
    {BLE_UUID_BMS_SERVICE_, m_bms.uuid_type}
  };

  //
  // Preset & Update Beacon settings
  //
  uint8_t *_beacon_info = ble_bms_get_beacon_info();

  if ( m_fcm == 0xFF || g_is_startup == 1 || (prev_status == 1 && g_is_startup == 0)) {
      build_bms_data();
  }
  prev_status = g_is_startup;

  //
  // Tangerine Data
  //
  ble_advdata_manuf_data_t tangerine_data;
  tangerine_data.company_identifier = TANGERINE_COMPANY_IDENTIFIER;
  tangerine_data.data.p_data        = (uint8_t *) bms_info;
  tangerine_data.data.size          = APP_BMS_INFO_LENGTH;

  // Build and set advertising data
  //
  memset(&advdata, 0, sizeof(advdata));
  advdata.name_type               = BLE_ADVDATA_NO_NAME;
  advdata.include_appearance      = false;
  advdata.flags                   = flags;
  advdata.p_manuf_specific_data   = &tangerine_data;

  //
  // Build and set advertising scan response
  //
  memset(&scanrsp, 0, sizeof(scanrsp));
  scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
  scanrsp.uuids_complete.p_uuids  = adv_uuids;

  err_code = ble_advdata_encode(&scanrsp, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
  APP_ERROR_CHECK(err_code);

  memcpy(&m_ble_adv_data, &m_adv_data, sizeof(ble_gap_adv_data_t));

  //
  // Set Timeslot Advertising PDU packet
  //
  if (ble_bms_get_timeslot_status() != 0x00) {
    radio_gap_adv_set_configure(&m_adv_data);
    return;
  }

  // Initialize advertising parameters (used when starting advertising).
  memset(&m_adv_params, 0, sizeof(m_adv_params));

  m_adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
  m_adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED; // Tangerine PacketはConnectable
  m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
  m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
  m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
  m_adv_params.duration        = 0;       // Never time out.

  err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
  APP_ERROR_CHECK(err_code);

  //
  // Set AdvData and ScanResponseData
  //
  err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
  APP_ERROR_CHECK(err_code);
  //
  // Update TxPower for Manager Packet
  //
  int8_t txPowerLevel = get_tx_power_level( _beacon_info[BINFO_TXPWR_FOR_MNG_IDX] ); // get TxPower for management
  err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, txPowerLevel);
  APP_ERROR_CHECK(err_code);
}
