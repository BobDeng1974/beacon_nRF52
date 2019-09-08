#include "ibeacon.h"

#define IBEACON_PDU_LENGTH        40    

static uint8_t  clbeacon_info[APP_CLBEACON_INFO_LENGTH] =         /**< Information advertised by the iBeacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this implementation.
    APP_IBEACON_UUID,    // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between iBeacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between iBeacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The iBeacon's measured TX power in this implementation.
};

static uint8_t  m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];    /**< Buffer for storing an encoded advertising set. */
static uint8_t  m_enc_scrdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];    /**< Buffer for storing an encoded scan response data set. */
static uint8_t  m_pdu_ibeacon[IBEACON_PDU_LENGTH];

/**@brief Building Tangerine iBeacon advertising data
*/
void build_ibeacon_data(void) 
{
  //
  // Preset & Update Beacon settings
  //
  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  
  // UUID/Major/Minor
  for (int i = 2; i < 18; i++) {
      clbeacon_info[i] = _beacon_info[i-2];
  }

  if (ble_tgsec_ibeacon_enablep() == 1) {
    clbeacon_info[18] = _beacon_info[BINFO_TGSECB_ROTATED_BEACONID_IDX];
    clbeacon_info[19] = _beacon_info[BINFO_TGSECB_ROTATED_BEACONID_IDX+1];
    clbeacon_info[20] = _beacon_info[BINFO_TGSECB_ROTATED_BEACONID_IDX+2];
    clbeacon_info[21] = _beacon_info[BINFO_TGSECB_ROTATED_BEACONID_IDX+3];
  }
  else {
    for (int i = 18; i < 20; i++) {
      clbeacon_info[i] = _beacon_info[i-2];
    }
    for (int i = 20; i < 22; i++) {
      clbeacon_info[i] = _beacon_info[i-2];
    }
  }

  // RSSI
  clbeacon_info[22] = get_adjusted_rssi(_beacon_info[BINFO_TXPWR_VALUE_IDX]); 

  // Battery status 
  // if you use this, define packet length as following. but this is iregular frame for standard iBeacon.
  //     #define APP_CLBEACON_INFO_LENGTH      0x18
#if defined(IBEACON_WITH_BATT_INFO)
  if (ble_tibeacon_pwinfop() == 1) {
    clbeacon_info[23] = get_status_flags();
  }
#endif 

}

/**@brief ibeacon initialization
 */
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

ble_gap_adv_data_t * get_ibeacon_adv_data(void)
{
  return &m_adv_data;
}

void ibeacon_advertising_init()
{
  uint32_t      err_code;
  ble_advdata_t advdata;
  //uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
  uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED | BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE;

  /* // RSSI */
  /* clbeacon_info[22] = get_adjusted_rssi(_beacon_info[21]); //_beacon_info[42]; */
  uint8_t *_beacon_info = ble_bms_get_beacon_info();

  //if (g_is_startup == 1)
  build_ibeacon_data();

  //
  // iBeacon Advertising Data
  //
  ble_advdata_manuf_data_t manuf_specific_data;
  manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
  manuf_specific_data.data.p_data        = (uint8_t *) clbeacon_info;
  manuf_specific_data.data.size          = APP_CLBEACON_INFO_LENGTH;

  //
  // Build and set advertising data
  //
  memset(&advdata, 0, sizeof(advdata));
  advdata.name_type               = BLE_ADVDATA_NO_NAME;
  advdata.include_appearance      = false;
  advdata.flags                   = flags;
  advdata.p_manuf_specific_data   = &manuf_specific_data;
  
  err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
  APP_ERROR_CHECK(err_code);

  //
  // Set Timeslot Advertising PDU packet
  //
  if (ble_bms_get_timeslot_status() != 0x00) {
    radio_gap_adv_set_configure(&m_adv_data);
    return;
  }

  //
  // Set AdvData and ScanResponseData
  //
  memset(&m_adv_params, 0, sizeof(m_adv_params));

  m_adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
  m_adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED; // Tangerine PacketはConnectable
  m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
  m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
  m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
  m_adv_params.duration        = 0;       // Never time out.

  err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
  APP_ERROR_CHECK(err_code);

  memcpy(&m_ble_adv_data, &m_adv_data, sizeof(ble_gap_adv_data_t));

  //
  // Update TxPower for iBeacon
  //
  int8_t txPowerLevel = get_tx_power_level( _beacon_info[BINFO_TXPWR_VALUE_IDX] );
  err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, txPowerLevel);
  APP_ERROR_CHECK(err_code);
}

uint8_t * get_ibeacon_advertising_data(void)
{
  return &m_enc_advdata;
}

/**@brief LINE Beacon Packet initialization
 */
void set_ibeacon_packet(void)
{
  uint32_t      err_code;
  ble_advdata_t advdata;
  uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED | BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE;

  /* // RSSI */
  uint8_t *_beacon_info = ble_bms_get_beacon_info();

  build_ibeacon_data();

  // iBeacon Advertising Data
  ble_advdata_manuf_data_t manuf_specific_data;
  manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
  manuf_specific_data.data.p_data        = (uint8_t *) clbeacon_info;
  manuf_specific_data.data.size          = APP_CLBEACON_INFO_LENGTH;

  // Build and set advertising data
  memset(&advdata, 0, sizeof(advdata));
  advdata.name_type               = BLE_ADVDATA_NO_NAME;
  advdata.include_appearance      = false;
  advdata.flags                   = flags;
  advdata.p_manuf_specific_data   = &manuf_specific_data;
  
  err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
  APP_ERROR_CHECK(err_code);

  // Set AdvData
  err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
  APP_ERROR_CHECK(err_code);

  // Set Timeslot Advertising PDU packet
  radio_pdu_configure(&m_adv_data, &m_adv_params, &m_pdu_ibeacon);
}

uint8_t * get_ibeacon_packet(void)
{
  return &m_pdu_ibeacon;
}
