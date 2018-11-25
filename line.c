#include "line.h"
#include "sha.h"

static uint8_t        m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                           /**< Buffer for storing an encoded advertising set. */
static uint8_t        m_enc_scrdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                           /**< Buffer for storing an encoded scan response data set. */

static uint8_t        data[LINE_SECMSG_ORIGIN_LENGTH];
static uint8_t        digest[SHA256HashSize];
static uint8_t        xors1st[16];
static uint8_t        xors2nd[8];
static uint8_t        xors3rd[4];

static SHA256Context  sha_context;

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


static uint8_t line_ibeacon_info[APP_CLBEACON_INFO_LENGTH] =                /**< Information advertised by the iBeacon. */
{
    APP_LINE_IBEACON_DEVICE_TYPE,      // Manufacturer specific information. Specifies the device type in this implementation.
    APP_LINE_IBEACON_ADV_DATA_LENGTH,  // Manufacturer specific information. Specifies the length of the manufacturer specific data in this implementation.
    APP_LINE_IBEACON_UUID,             // 128 bit UUID value.
    APP_LINE_IBEACON_MAJOR_VALUE,      // Major arbitrary value that can be used to distinguish between iBeacons.
    APP_LINE_IBEACON_MINOR_VALUE,      // Minor arbitrary value that can be used to distinguish between iBeacons.
    APP_LINE_IBEACON_MEASURED_RSSI     // Manufacturer specific information. The iBeacon's measured TX power in this implementation.
};

static uint8_t line_beacon_service_frame[APP_LINE_BEACON_SERVICE_FRAME_LENGTH] = /**< LINE Beacon pakcet service frame */
{
  LINE_BEACON_TYPE,                    // LINE Beacon type 0x02
  LINE_BEACON_HWID,                    // HWID
  LINE_BEACON_TXPW,                    // TxPower
  LINE_BEACON_SECMSG,                  // Secure Message
};

static  uint8_t secmsg[LINE_SECMSG_LENGTH];  /**< LINE Beacon Packet Secure Message */


/* private functions */
static void sha256_encoding(const uint8_t *in, unsigned int in_count, uint8_t *digest);
static bool xors_in_half(const uint8_t *in_data, unsigned int in_count, uint8_t *out);
static void build_line_secmsg(uint64_union_t timestamp, uint8_t *hwid, uint8_t *vendorKey, uint8_t *lotKey,  uint8_t bat, uint8_t *secmsg);


void line_sha256_init(void) 
{
  SHA256Context *psha_context = &sha_context;
  SHA256Reset(psha_context);
}

/**@brief Building LINE iBeacon advertising data
*/
void build_line_ibeacon_data(void) 
{
  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  
  // RSSI
  /* int8_t txpw = get_tx_power_level( _beacon_info[BINFO_TXPWR_VALUE_IDX] ); */
  /* if (txpw >= 0) { */
  /*   line_ibeacon_info[22] = txpw; */
  /* }  */
  /* else { */
  /*   line_ibeacon_info[22] = txpw + 256; */
  /* } */
  line_ibeacon_info[22] = get_adjusted_rssi(_beacon_info[BINFO_TXPWR_VALUE_IDX]); //_beacon_info[42];

}

/**@brief LINE ibeacon initialization
 */
void line_ibeacon_advertising_init()
{
  uint32_t      err_code;
  ble_advdata_t advdata;
  uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED | BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE;
  //uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

  uint8_t *_beacon_info = ble_bms_get_beacon_info();

  if (g_is_startup == 1)
    build_line_ibeacon_data();

  //
  // iBeacon Advertising Data
  //
  ble_advdata_manuf_data_t manuf_specific_data;
  manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
  manuf_specific_data.data.p_data        = (uint8_t *) line_ibeacon_info;
  manuf_specific_data.data.size          = APP_LINE_IBEACON_INFO_LENGTH;

  //
  // Build and set advertising data
  //
  memset(&advdata, 0, sizeof(advdata));
  advdata.name_type               = BLE_ADVDATA_NO_NAME;
  advdata.include_appearance      = false;
  advdata.flags                   = flags;
  advdata.p_manuf_specific_data   = &manuf_specific_data;
  
  m_adv_data.adv_data.len = 0x1f;     // Bug v41107 len=0x03 

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
  } else {

    //
    // Set AdvData and ScanResponseData
    //
    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);

    //
    // Update TxPower for iBeacon
    //
    int8_t txPowerLevel = get_tx_power_level( _beacon_info[BINFO_TXPWR_VALUE_IDX] );

    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, txPowerLevel);
    APP_ERROR_CHECK(err_code);
  }
}

/**@brief Building LINE Beacon Packet advertising data
*/
void build_line_beacon_packet_servicedata(void) 
{
  //static  uint8_t secmsg[LINE_SECMSG_LENGTH];
  memset(secmsg, 0, LINE_SECMSG_LENGTH);

  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  uint8_t *hwid         = &_beacon_info[BINFO_LINE_BEACON_HWID_IDX];
  uint8_t *vendorKey    = &_beacon_info[BINFO_LINE_BEACON_VENDOR_KEY_IDX];
  uint8_t *lotKey       = &_beacon_info[BINFO_LINE_BEACON_LOTKEY_IDX];
  uint8_t bat; 

  if (ble_batteryp() != 1) {
    bat = 0x00;
  }
  else {
    bat = _beacon_info[BINFO_BATTERY_LEVEL10_VALUE_IDX];
  }

  // HWID
  for (int i = 0; i < LINE_SECMSG_LENGTH; i++) {
    line_beacon_service_frame[1+i] = hwid[i];
  }

  // RSSI
  /* int8_t txpw = get_tx_power_level( _beacon_info[BINFO_TXPWR_VALUE_IDX] ); */
  /* if (txpw >= 0) { */
  /*   line_beacon_service_frame[6] = txpw; */
  /* }  */
  /* else { */
  /*   line_beacon_service_frame[6] = txpw + 256; */
  /* } */
  line_beacon_service_frame[6] = get_adjusted_rssi(_beacon_info[BINFO_TXPWR_VALUE_IDX]);  

  // SECMSG
  build_line_secmsg(m_line_timestamp, hwid, vendorKey, lotKey, bat, secmsg);
  for (int i = 0; i < LINE_SECMSG_LENGTH; i++) {
    line_beacon_service_frame[7+i] = secmsg[i];
  }

}

/**@brief LINE Beacon Packet initialization
 */
void line_beacon_packet_advertising_init()
{
  uint32_t      err_code;
  
  // service uuid
  uint16_union_t service_uuid;
  uint8_t uuid_tmp[] = {LINE_BEACON_SERVICE_UUID};
  service_uuid.array[0] = uuid_tmp[0];
  service_uuid.array[1] = uuid_tmp[1];

  // Ad Structure #1
  // Complete list of 16bit service UUID
  ble_uuid_t uuids[] = {{service_uuid.value, BLE_UUID_TYPE_BLE}};
  ble_advdata_uuid_list_t uuid_list = {1, uuids};

  ble_advdata_t advdata;
  uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED | BLE_GAP_ADV_FLAG_LE_GENERAL_DISC_MODE;
  //uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

  memset(&advdata, 0, sizeof(advdata));
  advdata.name_type               = BLE_ADVDATA_NO_NAME;
  advdata.include_appearance      = false;
  advdata.flags                   = flags;
  advdata.uuids_complete          = uuid_list;

  //if (g_is_startup == 1)
  build_line_beacon_packet_servicedata();

  // Ad Structure #2
  // Service Data
  ble_advdata_service_data_t service_data;
  
  service_data.service_uuid = service_uuid.value;

  uint8_array_t data = {APP_LINE_BEACON_SERVICE_FRAME_LENGTH, line_beacon_service_frame};
  service_data.data = data;
  advdata.p_service_data_array = &service_data;
  advdata.service_data_count = 1;

  // set adv frame and service frame to ble packet
  m_adv_data.adv_data.len = 0x1f;     // Bug v41107 len=0x03 
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
  } else {

    //
    // Set AdvData and ScanResponseData
    //
    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);

    //
    // Update TxPower
    //
    uint8_t *_beacon_info = ble_bms_get_beacon_info();
    int8_t txPowerLevel = get_tx_power_level( _beacon_info[BINFO_TXPWR_VALUE_IDX] );
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_adv_handle, txPowerLevel);
    APP_ERROR_CHECK(err_code);
  }
}

static void build_line_secmsg(uint64_union_t timestamp, uint8_t *hwid, uint8_t *vendorKey, uint8_t *lotKey,  uint8_t bat, uint8_t *secmsg) 
{
  memset(data, 0, LINE_SECMSG_ORIGIN_LENGTH);

  for (int i=0; i<8; i++) {
    data[i] = timestamp.array[7-i];
  }
  memcpy(data+8, hwid, LINE_HWID_LENGTH); 
  memcpy(data+13, vendorKey, LINE_VENDORKEY_LENGTH);
  memcpy(data+17, lotKey, LINE_LOTKEY_LENGTH);
  data[25] = bat;

  sha256_encoding(data, LINE_SECMSG_ORIGIN_LENGTH, digest);

  xors_in_half(digest, 32, xors1st);

  xors_in_half(xors1st, 16, xors2nd);

  xors_in_half(xors2nd, 8, xors3rd);

  memcpy(secmsg, xors3rd, 4);

  // sha256 + 3 xor 
  for(int i=0; i<2; i++) {
    secmsg[i+4] = timestamp.array[1-i];
  }

  // battery level
  secmsg[6] = bat;
}

static bool xors_in_half(const uint8_t *in_data, unsigned int in_count, uint8_t *out)
{
  if (in_count % 2 != 0) {
    return false;
  }

  unsigned int half = in_count / 2;

  for (int i = 0; i < half; i++) {
    out[i] = in_data[i] ^ in_data[half+i];
  }

  return true;
}

static void sha256_encoding(const uint8_t *in_data, unsigned int in_count, uint8_t *digest) 
{
  SHA256Context *psha_context = &sha_context;

  SHA256Reset(psha_context);   // line_sha256_init
  SHA256Input(psha_context, in_data, in_count);
  //SHA256FilnalBits(SHA256, ?, ?);
  SHA256Result(psha_context, digest);
}

