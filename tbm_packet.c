#include "tbm_packet.h"


static uint8_t bms_info[APP_BMS_INFO_LENGTH] =                  /**< Information advertised by BMS. */
{
    0x00, 0x00,                                                  // Service ID : 2 bytes
    0x00, 0x00, 0x00, 0x00,                                      // Serial ID : 4 bytes
    0x00, 0x00, 0x00, 0x00,                                      // Beacon ID : 4 bytes
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,              // Geo Hash : 8 bytes
    0x00,                                                        // Tx Power : 1 bytes
    0x00,                                                        // Battery / ECO : 1 bytes
    0x00,                                                        // Mode / Status 1 byte
    0x00,                                                        // Frequency 1 byte
    APP_FIRMWARE_VERSION_VALUE                                   // 2 bytes
}; // 24bytes

static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scrdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];    /**< Buffer for storing an encoded scan response data set. */

#define TBM_BEACON_PDU_LENGTH        40    
static  uint8_t m_pdu_tbm[TBM_BEACON_PDU_LENGTH];

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
  bms_info[0] = 0xD0;
  if (ble_line_beacon_enablep() == 1 || ble_tgsec_ibeacon_enablep() == 1) bms_info[0] = 0xC0;
  bms_info[1] = 0xFF;
  // 28-29
  //for (int i = BINFO_SERVICE_ID_VALUE_IDX; i < (BINFO_SERVICE_ID_VALUE_IDX+2); i++) {
  //  bms_info[i-28] = _beacon_info[i];
  //}

  // Serial ID: 2-5
  // 140-143 
  if (g_startup_stage == 1 && ble_tgsec_ibeacon_enablep() == 1) { 
    // order: mem: little endian, packet: big endian
    bms_info[2] = _beacon_info[BINFO_TGSECB_TIMESTAMP_IDX+3];
    bms_info[3] = _beacon_info[BINFO_TGSECB_TIMESTAMP_IDX+2];
    bms_info[4] = _beacon_info[BINFO_TGSECB_TIMESTAMP_IDX+1];
    bms_info[5] = _beacon_info[BINFO_TGSECB_TIMESTAMP_IDX+0];
  }
  else {
  // Iot Sensor Data 1
    bms_info[2] = 0x00;
    bms_info[3] = 0x00;
    bms_info[4] = 0x00;
    bms_info[5] = 0x00;
  }

  // Beacon ID (Major/Minor)
  for (int i = BINFO_MAJOR_VALUE_IDX; i < (BINFO_MAJOR_VALUE_IDX+4); i++) {
    bms_info[i-10] = _beacon_info[i];
    //print("%d - ", _beacon_info[i]);
  }
  if (g_startup_stage == 0) {
    bms_info[0] = 0xF0;
    if (ble_tgsec_ibeacon_enablep() == 1) {
      bms_info[0] = 0xE0;
      bms_info[6] = _beacon_info[BINFO_TGSECB_ROTATED_BEACONID_IDX];
      bms_info[7] = _beacon_info[BINFO_TGSECB_ROTATED_BEACONID_IDX+1];
      bms_info[8] = _beacon_info[BINFO_TGSECB_ROTATED_BEACONID_IDX+2];
      bms_info[9] = _beacon_info[BINFO_TGSECB_ROTATED_BEACONID_IDX+3];
    }
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
    uint16_t blevel = get_battery_level();
    bms_info[17] = (uint8_t)((blevel & 0xFF00) >> 8);
    bms_info[18] = (uint8_t)(blevel & 0x00FF);
  }

  // Tx Power
  bms_info[18] =  _beacon_info[BINFO_TXPWR_VALUE_IDX] & 0x0f;

  // Battery / ECO mode
  uint8_t statusFlags = _beacon_info[BINFO_BATTERY_LEVEL10_VALUE_IDX] & 0x7f;
  if ( m_hardware_type != HW_TYPE_TANGERINE_BEACON ) statusFlags = 0x7F;

  m_eco_start_time.dec.Hours     = _beacon_info[BINFO_ECO_MODE_START_TIME_IDX];
  m_eco_start_time.dec.Minutes   = _beacon_info[BINFO_ECO_MODE_START_TIME_IDX+1];
  m_eco_finish_time.dec.Hours    = _beacon_info[BINFO_ECO_MODE_FINISH_TIME_IDX];
  m_eco_finish_time.dec.Minutes  = _beacon_info[BINFO_ECO_MODE_FINISH_TIME_IDX+1];
  if (m_eco_start_time.wTime != m_eco_finish_time.wTime) statusFlags |= 0x08;
  bms_info[19] =  statusFlags;
  
  // Beacon Status
  statusFlags = m_timeslot_mode << 4;
  statusFlags = statusFlags | _beacon_info[BINFO_STATUS_VALUE_IDX];
  if (_beacon_info[BINFO_STATUS_VALUE_IDX] == 0x00) statusFlags = statusFlags | 0x08;
  bms_info[20] = statusFlags;

  // Tx Power / Beacon Frequency
  statusFlags = _beacon_info[BINFO_TIMESLOT_TXFRQ_VALUE_IDX];
  statusFlags = statusFlags  | (_beacon_info[BINFO_TXFRQ_VALUE_IDX] << 4);


  bms_info[21] = statusFlags;

  // Firmware Version
  bms_info[22] = _beacon_info[BINFO_VERSION_VALUE_IDX];
  bms_info[23] = _beacon_info[BINFO_VERSION_VALUE_IDX+1];

  // Hardware Type
  bms_info[0] = ( bms_info[0] & 0xF0) | m_hardware_type;
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

  build_bms_data();
  //if ( m_fcm == 0xFF || g_is_startup == 1 || (prev_status == 1 && g_is_startup == 0)) {
  //    build_bms_data();
  //}
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

  err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
  APP_ERROR_CHECK(err_code);

  memcpy(&m_ble_adv_data, &m_adv_data, sizeof(ble_gap_adv_data_t));

  //
  // Set Timeslot Advertising PDU packet
  //
  if (ble_bms_get_timeslot_status() != 0x00) {
    // Set Timeslot Advertising PDU packet
    radio_pdu_configure(&m_adv_data, &m_adv_params, &m_pdu_tbm);
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

uint8_t * get_bms_advertising_data(void)
{
  return &m_enc_advdata;
}
uint8_t * get_bms_info(void)
{
  return &bms_info[0];
}

uint8_t * get_tbm_packet(void)
{
  return &m_pdu_tbm;
}

