#include "eddystone.h"


// EDDYSTONE-URL1
static uint8_t eddystone_url_data1[] =   /**< Information advertised by the Eddystone URL frame type. */
{
    APP_EDDYSTONE_URL_FRAME_TYPE,   // Eddystone URL frame type.
    APP_EDDYSTONE_RSSI,             // RSSI value at 0 m.
    APP_EDDYSTONE_URL_SCHEME,       // Scheme or prefix for URL ("http", "http://www", etc.)
    APP_EDDYSTONE_URL_URL           // URL with a maximum length of 17 bytes. Last byte is suffix (".com", ".org", etc.)
};

// EDDYSTONE-URL2
static uint8_t eddystone_url_data2[] =   /**< Information advertised by the Eddystone URL frame type. */
{
    APP_EDDYSTONE_URL_FRAME_TYPE,   // Eddystone URL frame type.
    APP_EDDYSTONE_RSSI,             // RSSI value at 0 m.
    APP_EDDYSTONE_URL_SCHEME,       // Scheme or prefix for URL ("http", "http://www", etc.)
    APP_EDDYSTONE_URL_URL           // URL with a maximum length of 17 bytes. Last byte is suffix (".com", ".org", etc.)
};

// EDDYSTONE-UID
static uint8_t eddystone_uid_data[] =   /**< Information advertised by the Eddystone UID frame type. */
{
   APP_EDDYSTONE_UID_FRAME_TYPE,   // Eddystone UID frame type.
   APP_EDDYSTONE_RSSI,             // RSSI value at 0 m.
   APP_EDDYSTONE_UID_NAMESPACE,    // 10-byte namespace value. Similar to Beacon Major.
   APP_EDDYSTONE_UID_ID,           // 6-byte ID value. Similar to Beacon Minor.
   APP_EDDYSTONE_UID_RFU           // Reserved for future use.
};

// EDDYSTONE-TLM
static uint8_t eddystone_tlm_data[] =   /**< Information advertised by the Eddystone TLM frame type. */
{
   APP_EDDYSTONE_TLM_FRAME_TYPE,   // Eddystone TLM frame type.  [0]
   APP_EDDYSTONE_TLM_VERSION,      // Eddystone TLM version.     [1]
   APP_EDDYSTONE_TLM_BATTERY,      // Battery voltage in mV/bit. [2][3]
   APP_EDDYSTONE_TLM_TEMPERATURE,  // Temperature [C].           [4][5]
   APP_EDDYSTONE_TLM_ADV_COUNT,    // Number of advertisements since power-up or reboot. [6][7][8][9]
   APP_EDDYSTONE_TLM_SEC_COUNT     // Time since power-up or reboot. 0.1 s increments.   [10][11][12][13]
};

/**@brief Building EDDYSTONE-TLM advertising data
*/
void build_eddystone_tlm_data()
{
  // battery chaarge info
  eddystone_tlm_data[2] = m_battery_charge.array[1];
  eddystone_tlm_data[3] = m_battery_charge.array[0];

  // temperature
  // NOT SUPPORT!
  // eddystone_tlm_data[4];
  // eddystone_tlm_data[5];
  
  // adv count
  eddystone_tlm_data[6] = m_adv_count.array[3];
  eddystone_tlm_data[7] = m_adv_count.array[2];
  eddystone_tlm_data[8] = m_adv_count.array[1];
  eddystone_tlm_data[9] = m_adv_count.array[0];

  // sec count
  eddystone_tlm_data[10] = m_sec_count_100ms.array[3];
  eddystone_tlm_data[11] = m_sec_count_100ms.array[2];
  eddystone_tlm_data[12] = m_sec_count_100ms.array[1];
  eddystone_tlm_data[13] = m_sec_count_100ms.array[0];

}

/**@brief Building EDDYSTONE-UID advertising data
*/
void build_eddystone_uid_data()
{
  uint8_t *_beacon_info = ble_bms_get_beacon_info();

  eddystone_uid_data[1] = get_adjusted_rssi(_beacon_info[BINFO_TXPWR_VALUE_IDX]) + EDDYSTONE_URL_RSSI_CORRECTION;

  // Namespace
  for (int i = 0; i < BINFO_EDDYSTONE_UID_NAMESPACE_SIZ; ++i) {
    eddystone_uid_data[2+i] = _beacon_info[BINFO_EDDYSTONE_UID_NAMESPACE_IDX+i];
  }

  // ID
  for (int i = 0; i < BINFO_EDDYSTONE_UID_ID_SIZ; ++i) {
    eddystone_uid_data[12+i] = _beacon_info[BINFO_EDDYSTONE_UID_ID_IDX+i];
  }

}

/**@brief Building EDDYSTONE-URL advertising data
*/
static void build_eddystone_url_each_data(ble_advertising_mode_t url_mode)
{
  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  uint8_t *data;
  uint8_t info_index;

  if (url_mode == BLE_ADV_MODE_EDDYSTONE_URL1) {
    info_index = 0;
    data = eddystone_url_data1;
  }
  else if (url_mode == BLE_ADV_MODE_EDDYSTONE_URL2) {
    info_index = 20;
    data = eddystone_url_data2;
  }
  else {
    return;
  }

  data[1] = get_adjusted_rssi(_beacon_info[BINFO_TXPWR_VALUE_IDX]) + EDDYSTONE_URL_RSSI_CORRECTION;
  data[2] = _beacon_info[BINFO_EDDYSTONE1_URL_SCH_IDX+info_index]; // URL Scheme

  int len = _beacon_info[BINFO_EDDYSTONE1_URL_LEN_IDX+info_index];
  int index = 3;
  for (int i = 0; i < len; ++i) {
    data[3+i] = _beacon_info[BINFO_EDDYSTONE1_URL_URL_IDX+info_index+i];
    index++;
  }
  
  // concat battery string
  if (_beacon_info[BINFO_EDDYSTONE1_URL_BTC_IDX+info_index] == 0x01) {
    char* battery_string = get_battery_percent_string();
    for (int i = 0; i < 3; ++i) {
      data[index] = battery_string[i];
      index++;
    }
  }

}

void build_eddystone_url_data(void)
{
  build_eddystone_url_each_data(BLE_ADV_MODE_EDDYSTONE_URL1); // URL 1
  build_eddystone_url_each_data(BLE_ADV_MODE_EDDYSTONE_URL2); // URL 2
}

/**@Brief EDDYSTONE-URL advertising initialization
*/
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                           /**< Buffer for storing an encoded advertising set. */
static uint8_t              m_enc_scrdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                           /**< Buffer for storing an encoded scan response data set. */

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

void eddtystone_advertising_init(ble_advertising_mode_t eddystone_mode)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    //uint8_array_t   flags = {BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE};
    //uint8_array_t   flags = {BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED};

    ble_uuid_t    adv_uuids[] = {{APP_EDDYSTONE_UUID, BLE_UUID_TYPE_BLE}};

    uint8_array_t eddystone_data_array;                             // Array for Service Data structure.

    uint8_t *data;     // eddystone's data
    uint16_t size = 0; // size of eddystone's data

    // Preset & Update Beacon settings
    uint8_t *_beacon_info = ble_bms_get_beacon_info();

    // on statup stage, build all eddystone data
    if (g_is_startup == 1) {
      build_eddystone_url_data();
      build_eddystone_uid_data();
    }

    if (eddystone_mode == BLE_ADV_MODE_EDDYSTONE_URL1 || 
        eddystone_mode == BLE_ADV_MODE_EDDYSTONE_URL2) {
      uint16_t data_size;
      uint8_t info_index = 0;
      if (eddystone_mode == BLE_ADV_MODE_EDDYSTONE_URL1) {
        info_index = 0;
        data = eddystone_url_data1;
        data_size = sizeof(eddystone_url_data1);
      }
      else if (eddystone_mode == BLE_ADV_MODE_EDDYSTONE_URL2) {
        info_index = 20;
        data = eddystone_url_data2;
        data_size = sizeof(eddystone_url_data2);
      }

      uint8_t btc_len = 0;
      if (_beacon_info[BINFO_EDDYSTONE1_URL_BTC_IDX+info_index] == 0x01) {
        btc_len = 3;
      }
      else {
        btc_len = 0;
      }
      size = data_size - (17 - _beacon_info[44+info_index]) + btc_len;
    }
    else if (eddystone_mode == BLE_ADV_MODE_EDDYSTONE_UID) {
      size = sizeof(eddystone_uid_data);
      data = (uint8_t *)eddystone_uid_data;
    }
    else if (eddystone_mode == BLE_ADV_MODE_EDDYSTONE_TLM) {
      build_eddystone_tlm_data();
      size = sizeof(eddystone_tlm_data);
      data = eddystone_tlm_data;
    } 

    // Pointer to the data to advertise.
    eddystone_data_array.p_data = data;   

    // size of eddystone's data
    eddystone_data_array.size = size;

    ble_advdata_service_data_t service_data;                        // Structure to hold Service Data.
    service_data.service_uuid = APP_EDDYSTONE_UUID;                 // Eddystone UUID to allow discoverability on iOS devices.
    service_data.data = eddystone_data_array;                       // Array for service advertisement data.

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_NO_NAME;
    advdata.include_appearance      = false;

    advdata.flags                   = flags;

    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;
    advdata.p_service_data_array    = &service_data;                // Pointer to Service Data structure.
    advdata.service_data_count      = 1;

  err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
  APP_ERROR_CHECK(err_code);

  memset(&m_adv_params, 0, sizeof(m_adv_params));

  m_adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
  m_adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED; // Tangerine PacketはConnectable
  m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
  m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
  m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
  m_adv_params.duration        = 0;       // Never time out.

  //
  // Set Timeslot Advertising PDU packet
  //
  if (ble_bms_get_timeslot_status() != 0x00) {
    radio_gap_adv_set_configure(&m_adv_data, &m_adv_params);
  }

  //
  // Set AdvData and ScanResponseData
  //
  err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
  APP_ERROR_CHECK(err_code);

  memcpy(&m_ble_adv_data, &m_adv_data, sizeof(ble_gap_adv_data_t));

  //
  // Update TxPower for EddyStone
  //
  int8_t txPowerLevel = get_tx_power_level( _beacon_info[BINFO_TXPWR_VALUE_IDX] );
  err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, txPowerLevel);
  APP_ERROR_CHECK(err_code);
}

#define BATTEY_STR_LOW      "?b0" // Low: NOT USED
#define BATTEY_STR_25       "?b1" // 25%
#define BATTEY_STR_50       "?b2" // 50%
#define BATTEY_STR_75       "?b3" // 75%
#define BATTEY_STR_100      "?b4" // 100%
#define BATTEY_STR_CHARGING "?b5" // Charging: NOT USED

char* get_battery_percent_string()
{
  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  uint8_t bat = (_beacon_info[BINFO_BATTERY_VALUE_IDX] & 0b01110000) >> 4;

  switch (bat){
  case 0:
    return BATTEY_STR_LOW; // 0%
    break;
  case 1:
    return BATTEY_STR_25; // 25%
    break;
  case 2:
    return BATTEY_STR_50; // 50%
    break;
  case 3:
    return BATTEY_STR_75; // 75%
    break;
  case 4:
    return BATTEY_STR_100; // 100%
    break;
  default:
    return BATTEY_STR_LOW;
  }
}
