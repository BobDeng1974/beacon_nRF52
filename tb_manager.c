#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nordic_common.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_soc.h"

#include "tb_manager.h"
#include "advertising.h"

#define BNS_DEFAULT_IBEACON_UUID              0x01, 0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89, 0x9A, 0xAB, 0xBC, 0xCD, 0xDE, 0xEF, 0xF0
#define BMS_DEFAULT_MAJOR_VALUE               0x00, 0x00                        /**< Major value used to identify iBeacons. */
#define BMS_DEFAULT_MINOR_VALUE               0x00, 0x00                        /**< Minor value used to identify iBeacons. */


#define BMS_DEFAULT_STATUS_VALUE              0x03
#define BMS_DEFAULT_TXPWR_VALUE               0x06 // default TxPower : -4dBm
#define BMS_DEFAULT_TXFRQ_VALUE               0x02 // default Interval :  400msec
#define BMS_DEFAULT_BATTERY_VALUE             0x00

#define BMS_DEFAULT_SERIAL_ID_VALUE           0x00, 0x00, 0x00, 0x00
#define BMS_DEFAULT_SERVICE_ID_VALUE          0x00, 0x00
#define BMS_DEFAULT_GEO_HASH_VALUE            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
#define BMS_DEFAULT_VERSION_VALUE             0x00, 0x00
#define BMS_DEFAULT_RSSI_VALUE                0x00 // +-0

// EDDYSTONE-URL
#define BMS_DEFAULT_EDDYSTONE_URL_LEN         0x0D 
#define BMS_DEFAULT_EDDYSTONE_URL_URL         0x67, 0x6f, 0x6f, 0x2e, 0x67, 0x6c, 0x2f, \
                                              0x51, 0x42, 0x4c, 0x39, 0x72, 0x57, \
                                              0x00, 0x00, 0x00, 0x00
#define BMS_DEFAULT_EDDYSTONE_URL_SCH         0x03  // "https://" : 0x03
#define BMS_DEFAULT_EDDYSTONE_URL_BTC         0x00  // NOT add battery character by default.

#define BMS_DEFAULT_MODE_LIST                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00     // advertising mode list

// ibeacon/eddystone-url/eddystone-uid enabled/disabled
#define BMS_IBEACON_ENABLED                   0x01
#define BMS_EDDSTONE_URL_ENABLED              0x02
#define BMS_EDDSTONE_UID_ENABLED              0x04
#define BMS_EDDSTONE_TLM_ENABLED              0x08
#define BMS_LINE_BEACON_ENABLED               0x10
#define BMS_TGSEC_IBEACON_ENABLED             0x20
#define BMS_DEFAULT_ADVERTISE_SETTING         (BMS_IBEACON_ENABLED)
#define BMS_DEFAULT_TXPWR_FOR_MNG            0x06 // default TxPower for Mangement : -4dBm

// EddyStone-UID
#define BMS_DEFAULT_EDDYSTONE_UID_ID         0x01, 0x02, 0x03, 0x04, \
                                             0x05, 0x06                        /**< Mock values for 6-byte Eddystone UID
                                                                             ID instance.  */
#define BMS_DEFAULT_EDDYSTONE_UID_NAMESPACE  0xAA, 0xAA, 0xBB, 0xBB, 0xCC, 0xCC, 0xDD, 0xDD, \
                                             0xEE, 0xEE                        /**< Mock values for 10-byte Eddystone UID  
                                                                                Namespace. */

#define BMS_DEFAULT_LINE_BEACON_HWID         0xA0, 0xDC, 0xED, 0x01, 0x2B                    /**< LINE BEACON HWID */
#define BMS_DEFAULT_LINE_BEACON_VENDOR_KEY   0x5C, 0xF2, 0xA4, 0x23                          /**< LINE BEACON VendorKey */
#define BMS_DEFAULT_LINE_BEACON_LOTKEY       0x8C, 0x19, 0x4F, 0xE4, 0x1D, 0x7F, 0xE3, 0x4F  /**< LINE BEACON LotKey */

#define BMS_DEFAULT_BATTERY_LEVEL10_VALUE   0x00 // Battery Level divided by 10

#define BMS_PW_CONT_UNKNOWN                 0x01 // power source is continuous or unkown
#define BMS_TIBEACON_PWINFO                 0x02 // Tangerine iBeacon includes power info
#define BMS_DEFAULT_SYSCONFIG_FLAGS         0x02 // System configuration flags: power is battery and Tangerine iBeacon not include power info

#define BMS_15SEC_TIMESTAMP                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // timestamp for 15sec counter

#define BMS_TGSECB_TIMESTAMP                0x00, 0x00, 0x00, 0x00             // tg secure timestamp
#define BMS_TGSECB_SEC_KEY                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // tg secure secure key

#define BMS_TGSECB_ROTATED_BEACONID         0x00, 0x00, 0x00, 0x00             // tg secure rotated beaconID

#define BMS_SET_CURRENT_DATETIME            0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x18, 0x00  // sec/min/hour/days/week/mon/year

#define BMS_ECO_MODE_START_TIME             0x00, 0x00                         // Eco mode start time
#define BMS_ECO_MODE_FINISH_TIME            0x00, 0x00                         // Eco mode finish time

#define BMS_CURRENT_DATETIME                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // sec/min/hour/days/week/mon/year

#define BMS_TIMESLOT_MODE_STATUS            0x81, 0x00                         // ECO mode status
#define BMS_TIMESLOT_TXFRQ_VALUE            0x0F, 0x00                         // Timeslot mode Intervals

#define BMS_TBM_TXFRQ_VALUE                 0x0C, 0x00                         // BMS advertising Intervals

#define BMS_BATTERY_MAX_CAPACITY            0xD4, 0x01                         // Battery Maximum capacity

#define BMS_HARDWARE_TYPE                   0x00, 0x00                         // Hardware Type

// flag operation
#define BMS_FLG_GET(flg, bits)               (flg & bits)
#define BMS_FLG_SET(flg, bits)               (flg | bits)
#define BMS_FL_CLR(flg, bits)                (flg & ~bits)

/* Persistent Beacon Settings  */
#define BMS_BEACON_INFO_LENGTH                  190

static uint8_t m_beacon_info[BMS_BEACON_INFO_LENGTH] = {

    BNS_DEFAULT_IBEACON_UUID,            // l=16 : s=0
    BMS_DEFAULT_MAJOR_VALUE,             // l=2  : s=16
    BMS_DEFAULT_MINOR_VALUE,             // l=2  : s=18

    BMS_DEFAULT_STATUS_VALUE,            // l=1  : s=20

    BMS_DEFAULT_TXPWR_VALUE,             // l=1  : s=21
    BMS_DEFAULT_TXFRQ_VALUE,             // l=1  : s=22

    BMS_DEFAULT_BATTERY_VALUE,           // l=1  : s=23
    BMS_DEFAULT_SERIAL_ID_VALUE,         // l=4  : s=24
    BMS_DEFAULT_SERVICE_ID_VALUE,        // l=2  : s=28
    BMS_DEFAULT_GEO_HASH_VALUE,          // l=10 : s=30

    BMS_DEFAULT_VERSION_VALUE,           // l=2  : s=40

    BMS_DEFAULT_RSSI_VALUE,              // l=1  : s=42

    BMS_DEFAULT_ADVERTISE_SETTING,       // l=1  : s=43

    BMS_DEFAULT_EDDYSTONE_URL_LEN,       // l=1  : s=44 : EDDYSTONE-URL1
    BMS_DEFAULT_EDDYSTONE_URL_URL,       // l=17 : s=45 : EDDYSTONE-URL1
    BMS_DEFAULT_EDDYSTONE_URL_SCH,       // l=1  : s=62 : EDDYSTONE-URL1
    BMS_DEFAULT_EDDYSTONE_URL_BTC,       // l=1  : s=63 : EDDYSTONE-URL1

    BMS_DEFAULT_EDDYSTONE_URL_LEN,       // l=1  : s=64 : EDDYSTONE-URL2
    BMS_DEFAULT_EDDYSTONE_URL_URL,       // l=17 : s=65 : EDDYSTONE-URL2
    BMS_DEFAULT_EDDYSTONE_URL_SCH,       // l=1  : s=82 : EDDYSTONE-URL2
    BMS_DEFAULT_EDDYSTONE_URL_BTC,       // l=1  : s=83 : EDDYSTONE-URL2
    
    BMS_DEFAULT_MODE_LIST,               // l=12 : s=84 

    BMS_DEFAULT_TXPWR_FOR_MNG,           // l=1  : s=96 : TxPower for Mangement 

    BMS_DEFAULT_EDDYSTONE_UID_ID,        // l=6  : s=97  : EDDYSTONE-UID
    BMS_DEFAULT_EDDYSTONE_UID_NAMESPACE, // l=10 : s=103 : EDDYSTONE-UID
    
    BMS_DEFAULT_LINE_BEACON_HWID,        // l=5 : s=113 : LINE Beacon 
    BMS_DEFAULT_LINE_BEACON_VENDOR_KEY,  // l=4 : s=118 : LINE Beacon
    BMS_DEFAULT_LINE_BEACON_LOTKEY,      // l=8 : s=122 : LINE BEacon

    BMS_DEFAULT_BATTERY_LEVEL10_VALUE,   // l=1 : s=130 : Battery Level divided by 10
    BMS_DEFAULT_SYSCONFIG_FLAGS,         // l=1 : s=131 : Battery source

    BMS_15SEC_TIMESTAMP,                 // l=8 : s=132 : 15sec timestamp

    BMS_TGSECB_TIMESTAMP,                // l=4 : s=140 : tg secure timestamp
    BMS_TGSECB_SEC_KEY,                  // l=6 : s=144 : tg secure secure key

    BMS_TGSECB_ROTATED_BEACONID,         // l=4 : s=150 : tg secure rotated BeaconID

    BMS_SET_CURRENT_DATETIME,            // l=8 : s=154 : ECO mode status

    BMS_ECO_MODE_START_TIME,             // l=2 : s=162 : Eco mode start time
    BMS_ECO_MODE_FINISH_TIME,            // l=2 : s=164 : Eco mode finish time

    BMS_CURRENT_DATETIME,                // l=8 : s=166 : ECO mode status

    BMS_TIMESLOT_MODE_STATUS,            // l=12 : s=174 : Timeslot mode Intervals
    BMS_TIMESLOT_TXFRQ_VALUE,            // l=12 : s=176 : Timeslot mode Intervals

    BMS_TBM_TXFRQ_VALUE,                 // l=2  : s=178 : BMS advertising Intervals
  
    BMS_BATTERY_MAX_CAPACITY,            // l=2  : s=180 : Battery Maximum capacity

    BMS_HARDWARE_TYPE                    // l=2  : s=182 : HARDWARE TYPE

};

#define BMS_DB_SIZE \
CEIL_DIV(sizeof(uint8_t) * BMS_BEACON_INFO_LENGTH, sizeof(uint32_t))  /**< Size of bonded centrals database in word size (4 byte). */

void ble_bms_set_default_value_to_beacon_info();

/* File ID and Key used for the configuration record. */

#define CONFIG_FILE     (0xF010)
#define CONFIG_REC_KEY  (0x7010)

/* Array to map FDS return values to strings. */
char const * fds_err_str[] =
{
    "FDS_SUCCESS",
    "FDS_ERR_OPERATION_TIMEOUT",
    "FDS_ERR_NOT_INITIALIZED",
    "FDS_ERR_UNALIGNED_ADDR",
    "FDS_ERR_INVALID_ARG",
    "FDS_ERR_NULL_ARG",
    "FDS_ERR_NO_OPEN_RECORDS",
    "FDS_ERR_NO_SPACE_IN_FLASH",
    "FDS_ERR_NO_SPACE_IN_QUEUES",
    "FDS_ERR_RECORD_TOO_LARGE",
    "FDS_ERR_NOT_FOUND",
    "FDS_ERR_NO_PAGES",
    "FDS_ERR_USER_LIMIT_REACHED",
    "FDS_ERR_CRC_CHECK_FAILED",
    "FDS_ERR_BUSY",
    "FDS_ERR_INTERNAL",
};

/* Array to map FDS events to strings. */
static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};

/* Keep track of the progress of a delete_all operation. */
static struct
{
    bool delete_next;   //!< Delete next record.
    bool pending;       //!< Waiting for an fds FDS_EVT_DEL_RECORD event, to delete the next record.
} m_delete_all;

/* Flag to check fds initialization. */
static bool volatile m_fds_initialized;

//
// public functions
//
static void fds_evt_handler(fds_evt_t const * p_evt)
{
    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == FDS_SUCCESS)
            {
                m_fds_initialized = true;
            }
            break;

        case FDS_EVT_WRITE:
          break;

        case FDS_EVT_DEL_RECORD:
        {
            m_delete_all.pending = false;
        } break;

        default:
            break;
    }
}

/**@brief   Begin deleting all records, one by one. */
void delete_all_begin(void)
{
    m_delete_all.delete_next = true;
}


/**@brief   Process a delete all command.
 *
 * Delete records, one by one, until no records are left.
 */
void delete_all_process(void)
{
    if (   m_delete_all.delete_next
        & !m_delete_all.pending)
    {

        m_delete_all.delete_next = record_delete_next();
    }
}

/**@brief   Wait for fds to initialize. */
static void wait_for_fds_ready(void)
{
    while (!m_fds_initialized)
    {
      nrf_pwr_mgmt_run();
    }
}

void tb_manager_pstorage_init(void)
{
  uint32_t  err_code;

  // Register first to receive an event when initialization is complete.
  (void) fds_register(fds_evt_handler);

  err_code = fds_init();
  APP_ERROR_CHECK(err_code);

  // Wait for fds to initialize.
  wait_for_fds_ready();
 
  fds_stat_t stat = {0};
  err_code = fds_stat(&stat);
  APP_ERROR_CHECK(err_code);

}


uint32_t tb_manager_init()
{
    uint32_t  err_code;

    // Load stored settings
    err_code = tb_manager_settings_load();

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t tb_manager_settings_load(void)
{
    ret_code_t err_code;
    fds_record_desc_t desc = {0};
    fds_find_token_t  token = {0};

    CRITICAL_REGION_ENTER();

    err_code = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &token);
    if (err_code == FDS_SUCCESS)
    {
        fds_flash_record_t record = {0};
        err_code = fds_record_open(&desc, &record);
        if (err_code == NRF_SUCCESS)
        {
            memcpy(&m_beacon_info, record.p_data, sizeof(m_beacon_info));
            err_code = fds_record_close(&desc);
        }
    }
    else if (err_code == FDS_ERR_NOT_FOUND)
    {
        memset(&m_beacon_info, 0xFF, sizeof(m_beacon_info));
        err_code = NRF_SUCCESS;
        ble_bms_set_default_value_to_beacon_info();
    }
    else
    {
        err_code = NRF_ERROR_INTERNAL;
    }

    CRITICAL_REGION_EXIT();

    return err_code;
}


uint32_t tb_manager_settings_store(void)
{
    ret_code_t err_code;
    fds_record_desc_t desc = {0};
    fds_find_token_t  token = {0};

    fds_record_t      record = {
            .file_id = CONFIG_FILE,
            .key     = CONFIG_REC_KEY,
            .data = {
                    .p_data       = &m_beacon_info,
                    .length_words = (BMS_DB_SIZE * sizeof(uint32_t))
            }
    };

    CRITICAL_REGION_ENTER();

    err_code = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &token);
    if (err_code == FDS_SUCCESS)
    {
        err_code = fds_record_update(&desc, &record);
    }
    else if (err_code == FDS_ERR_NOT_FOUND)
    {
        err_code = fds_record_write(&desc, &record);
    }
    else
    {
        err_code = NRF_ERROR_INTERNAL;
    }

    if (err_code == FDS_ERR_NO_SPACE_IN_FLASH)
    {
        // If there is no space, preserve write request and call Garbage Collector.
        err_code = fds_gc();
    }

    CRITICAL_REGION_EXIT();

    return err_code;
}


uint32_t tb_manager_reset(void)
{
    ret_code_t err_code;
    fds_record_desc_t desc = {0};
    fds_find_token_t  token = {0};

    fds_record_t      record = {
            .file_id = CONFIG_FILE,
            .key     = CONFIG_REC_KEY,
            .data = {
                    .p_data       = &m_beacon_info,
                    .length_words = (BMS_DB_SIZE * sizeof(uint32_t))
            }
    };

    CRITICAL_REGION_ENTER();

    memset(&m_beacon_info, 0xFF, sizeof(m_beacon_info));
    ble_bms_set_default_value_to_beacon_info();

    err_code = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &token);
    if (err_code == FDS_SUCCESS)
    {
        err_code = fds_record_delete(&desc);	
    }

    CRITICAL_REGION_EXIT();

    return err_code;
}


uint8_t* ble_bms_get_beacon_info(void)
{
    return (uint8_t *) &m_beacon_info;
}

uint8_t ble_ibeacon_enablep() 
{
  uint8_t bAdvSetting = m_beacon_info[ADVERTISE_SETTING_IDX];
  
  if (ble_bms_get_timeslot_status() != 0x00) bAdvSetting = m_advertising_packet_type;

  if (BMS_FLG_GET(bAdvSetting, BMS_IBEACON_ENABLED) != 0) {
    return 1;
  }
  else {
    return 0;
  }
}

uint8_t ble_eddystone_url1_enablep() 
{
  if ((BMS_FLG_GET(m_beacon_info[ADVERTISE_SETTING_IDX], BMS_EDDSTONE_URL_ENABLED) != 0) &&
    m_beacon_info[BINFO_EDDYSTONE1_URL_LEN_IDX] != 0) {
    return 1;
  }
  else {
    return 0;
  }
}

uint8_t ble_eddystone_url2_enablep() 
{
  if ((BMS_FLG_GET(m_beacon_info[ADVERTISE_SETTING_IDX], BMS_EDDSTONE_URL_ENABLED) != 0) &&
    m_beacon_info[BINFO_EDDYSTONE2_URL_LEN_IDX] != 0) {
    return 1;
  }
  else {
    return 0;
  }
}

uint8_t ble_eddystone_uid_enablep() 
{
  if (BMS_FLG_GET(m_beacon_info[ADVERTISE_SETTING_IDX], BMS_EDDSTONE_UID_ENABLED) != 0) {
    return 1;
  }
  else {
    return 0;
  }
}

uint8_t ble_eddystone_tlm_enablep() 
{
  if (BMS_FLG_GET(m_beacon_info[ADVERTISE_SETTING_IDX], BMS_EDDSTONE_TLM_ENABLED) != 0) {
    return 1;
  }
  else {
    return 0;
  }
}

uint8_t ble_line_beacon_enablep() 
{
  uint8_t bAdvSetting = m_beacon_info[ADVERTISE_SETTING_IDX];
  
  if (ble_bms_get_timeslot_status() != 0x00) bAdvSetting = m_advertising_packet_type;

  if (BMS_FLG_GET(bAdvSetting, BMS_LINE_BEACON_ENABLED) != 0) {
    return 1;
  }
  else {
    return 0;
  }
}

uint8_t ble_tgsec_ibeacon_enablep() 
{
  uint8_t bAdvSetting = m_beacon_info[ADVERTISE_SETTING_IDX];
  
  if (ble_bms_get_timeslot_status() != 0x00) bAdvSetting = m_advertising_packet_type;

  if (BMS_FLG_GET(bAdvSetting, BMS_TGSEC_IBEACON_ENABLED) != 0) {
    return 1;
  }
  else {
    return 0;
  }
}

uint8_t ble_batteryp() 
{
  if (BMS_FLG_GET(m_beacon_info[BINFO_SYSCONFIG_FLAGS_IDX], BMS_PW_CONT_UNKNOWN) == 0) {
    return 1;
  }
  else {
    return 0;
  }
}

uint8_t ble_tibeacon_pwinfop() 
{
  if (BMS_FLG_GET(m_beacon_info[BINFO_SYSCONFIG_FLAGS_IDX], BMS_TIBEACON_PWINFO) != 0) {
    return 1;
  }
  else {
    return 0;
  }
}

void ble_bms_reset_beacon_info()
{
  for (int i = 0; i < BMS_BEACON_INFO_LENGTH; ++i) {
    m_beacon_info[i] = 0xFF;
  }

  ble_bms_set_default_value_to_beacon_info();

}

void ble_bms_set_default_value_to_beacon_info()
{
  if (m_beacon_info[BINFO_MAJOR_VALUE_IDX] == 0xFF) {
    m_beacon_info[BINFO_MAJOR_VALUE_IDX] = 0x00;    
    m_beacon_info[BINFO_MAJOR_VALUE_IDX+1] = 0x00;            
  }
  if (m_beacon_info[BINFO_MINOR_VALUE_IDX] == 0xFF) {
    m_beacon_info[BINFO_MINOR_VALUE_IDX]   = 0x00;
    m_beacon_info[BINFO_MINOR_VALUE_IDX+1] = 0x00;        
  }
  
  // 初期値を上書きする
  if (m_beacon_info[BINFO_STATUS_VALUE_IDX] == 0xFF) {
    // ステータス
    m_beacon_info[BINFO_STATUS_VALUE_IDX] = 0x00; // 未登録
    //m_beacon_info[BINFO_STATUS_VALUE_IDX] = 0x04; // active ; for test
  }
  if (m_beacon_info[BINFO_TXPWR_VALUE_IDX] == 0xFF) {
    // Tx Power
    m_beacon_info[BINFO_TXPWR_VALUE_IDX] = BMS_DEFAULT_TXPWR_VALUE;
  }
  if (m_beacon_info[BINFO_TXFRQ_VALUE_IDX] == 0xFF) {
    // Tx Frequency
    m_beacon_info[BINFO_TXFRQ_VALUE_IDX] = BMS_DEFAULT_TXFRQ_VALUE;
  }
  if (m_beacon_info[BINFO_BATTERY_VALUE_IDX] == 0xFF) {
    // Battery
    m_beacon_info[BINFO_BATTERY_VALUE_IDX] = BMS_DEFAULT_BATTERY_VALUE;
  }
  if (m_beacon_info[BINFO_RSSI_VALUE_IDX] == 0xFF) {
    // RSSI
    m_beacon_info[BINFO_RSSI_VALUE_IDX] = BMS_DEFAULT_RSSI_VALUE;
  }

  if (m_beacon_info[ADVERTISE_SETTING_IDX] == 0xFF) {
    // Advertising setting
    memset(&m_beacon_info[ADVERTISE_SETTING_IDX], 0 , 1);
    m_beacon_info[ADVERTISE_SETTING_IDX] = BMS_DEFAULT_ADVERTISE_SETTING;
  }

  // EddyStone-URL 1
  if (m_beacon_info[BINFO_EDDYSTONE1_URL_LEN_IDX] == 0xFF) {
    // URL length
    m_beacon_info[BINFO_EDDYSTONE1_URL_LEN_IDX] = 0x00;
  }
  if (m_beacon_info[BINFO_EDDYSTONE1_URL_SCH_IDX] == 0xFF) {
    // URL Scheme
    m_beacon_info[BINFO_EDDYSTONE1_URL_SCH_IDX] = BMS_DEFAULT_EDDYSTONE_URL_SCH;
  }
  if (m_beacon_info[BINFO_EDDYSTONE1_URL_BTC_IDX] == 0xFF) {
    // URL Battery checking character
    m_beacon_info[BINFO_EDDYSTONE1_URL_BTC_IDX] = BMS_DEFAULT_EDDYSTONE_URL_BTC;
  }

  // EddyStone-URL 2
  if (m_beacon_info[BINFO_EDDYSTONE2_URL_LEN_IDX] == 0xFF) {
    // URL length
    m_beacon_info[BINFO_EDDYSTONE2_URL_LEN_IDX] = 0x00;
  }
  if (m_beacon_info[BINFO_EDDYSTONE2_URL_SCH_IDX] == 0xFF) {
    // URL Scheme
    m_beacon_info[BINFO_EDDYSTONE2_URL_SCH_IDX] = BMS_DEFAULT_EDDYSTONE_URL_SCH;
  }
  if (m_beacon_info[BINFO_EDDYSTONE_URL_BTC_IDX] == 0xFF) {
    // URL Battery checking character
    m_beacon_info[BINFO_EDDYSTONE_URL_BTC_IDX] = BMS_DEFAULT_EDDYSTONE_URL_BTC;
  }

  if (m_beacon_info[BINFO_MODE_LIST_IDX] == 0xFF) {
    // mode list
    /* m_beacon_info[BINFO_MODE_LIST_IDX]   = BLE_ADV_MODE_BMS; */
    /* m_beacon_info[BINFO_MODE_LIST_IDX+1] = BLE_ADV_MODE_IBEACON; */
    /* m_beacon_info[BINFO_MODE_LIST_IDX+2] = BLE_ADV_MODE_EDDYSTONE_URL1; */
    /* m_beacon_info[BINFO_MODE_LIST_IDX+3] = BLE_ADV_MODE_EDDYSTONE_URL2; */
    /* m_beacon_info[BINFO_MODE_LIST_IDX+4] = BLE_ADV_MODE_EDDYSTONE_UID; */
    /* m_beacon_info[BINFO_MODE_LIST_IDX+5] = BLE_ADV_MODE_EDDYSTONE_TLM; */
    /* m_beacon_info[BINFO_MODE_LIST_IDX+6] = BLE_ADV_MODE_LINE_IBEACON; */
    /* m_beacon_info[BINFO_MODE_LIST_IDX+6] = BLE_ADV_MODE_LINE_PACKET; */
    /* for (int i = BINFO_MODE_LIST_IDX+7; i < BINFO_MODE_LIST_IDX+ BINFO_MODE_LIST_SIZ ; ++i) { */
    /*   m_beacon_info[i] = BLE_ADV_MODE_UNDEFINED; */
    /* } */

    // Tangerine iBeacon TEST
    //m_beacon_info[BINFO_MODE_LIST_IDX] = BLE_ADV_MODE_BMS;
    //m_beacon_info[BINFO_MODE_LIST_IDX+1] = BLE_ADV_MODE_IBEACON;

    // FOR LINE BEACON TEST
    //m_beacon_info[BINFO_MODE_LIST_IDX] = BLE_ADV_MODE_LINE_IBEACON;
    //m_beacon_info[BINFO_MODE_LIST_IDX+1] = BLE_ADV_MODE_LINE_PACKET;

    // Tangerine Secure iBeacon TEST
    m_beacon_info[BINFO_MODE_LIST_IDX] = BLE_ADV_MODE_IBEACON;
    
    for (int i = BINFO_MODE_LIST_IDX+1; i < BINFO_MODE_LIST_IDX + BINFO_MODE_LIST_SIZ ; ++i) {
      m_beacon_info[i] = BLE_ADV_MODE_UNDEFINED;
    }
  }

  if (m_beacon_info[BINFO_TXPWR_FOR_MNG_IDX] == 0xFF) {
    // TxPower for management
    m_beacon_info[BINFO_TXPWR_FOR_MNG_IDX] = BMS_DEFAULT_TXPWR_FOR_MNG;
  }

  if (m_beacon_info[97] == 0xFF) {
    // EddyStone-UID ID
    m_beacon_info[BINFO_EDDYSTONE_UID_ID_IDX]   = 0x07;
    m_beacon_info[BINFO_EDDYSTONE_UID_ID_IDX+1] = 0x02;
    m_beacon_info[BINFO_EDDYSTONE_UID_ID_IDX+2] = 0x03;
    m_beacon_info[BINFO_EDDYSTONE_UID_ID_IDX+3] = 0x04;
    m_beacon_info[BINFO_EDDYSTONE_UID_ID_IDX+4] = 0x05;
    m_beacon_info[BINFO_EDDYSTONE_UID_ID_IDX+5] = 0x06;
  }

  if (m_beacon_info[BINFO_EDDYSTONE_UID_NAMESPACE_IDX] == 0xFF) {
    // EddyStone-UID for "tangerine.io"
    m_beacon_info[BINFO_EDDYSTONE_UID_NAMESPACE_IDX]   = 0x13;
    m_beacon_info[BINFO_EDDYSTONE_UID_NAMESPACE_IDX+1] = 0x18;
    m_beacon_info[BINFO_EDDYSTONE_UID_NAMESPACE_IDX+2] = 0xfa;
    m_beacon_info[BINFO_EDDYSTONE_UID_NAMESPACE_IDX+3] = 0xba;
    m_beacon_info[BINFO_EDDYSTONE_UID_NAMESPACE_IDX+4] = 0x43;
    m_beacon_info[BINFO_EDDYSTONE_UID_NAMESPACE_IDX+5] = 0xf2;
    m_beacon_info[BINFO_EDDYSTONE_UID_NAMESPACE_IDX+6] = 0x55;
    m_beacon_info[BINFO_EDDYSTONE_UID_NAMESPACE_IDX+7] = 0x0e;
    m_beacon_info[BINFO_EDDYSTONE_UID_NAMESPACE_IDX+8] = 0x0f;
    m_beacon_info[BINFO_EDDYSTONE_UID_NAMESPACE_IDX+9] = 0x22;
  }
  
  if (m_beacon_info[BINFO_LINE_BEACON_HWID_IDX] == 0xFF) { 
    m_beacon_info[BINFO_LINE_BEACON_HWID_IDX]   = 0x00;
    m_beacon_info[BINFO_LINE_BEACON_HWID_IDX+1] = 0x00;
    m_beacon_info[BINFO_LINE_BEACON_HWID_IDX+2] = 0x00;
    m_beacon_info[BINFO_LINE_BEACON_HWID_IDX+3] = 0x00;
    m_beacon_info[BINFO_LINE_BEACON_HWID_IDX+4] = 0x00;
  }

  if (m_beacon_info[BINFO_LINE_BEACON_VENDOR_KEY_IDX] == 0xFF) {
    m_beacon_info[BINFO_LINE_BEACON_VENDOR_KEY_IDX]   = 0x5C;
    m_beacon_info[BINFO_LINE_BEACON_VENDOR_KEY_IDX+1] = 0xF2;
    m_beacon_info[BINFO_LINE_BEACON_VENDOR_KEY_IDX+2] = 0xA4;
    m_beacon_info[BINFO_LINE_BEACON_VENDOR_KEY_IDX+3] = 0x23;
  }

  if (m_beacon_info[BINFO_LINE_BEACON_LOTKEY_IDX] == 0xFF) {
    m_beacon_info[BINFO_LINE_BEACON_LOTKEY_IDX]   = 0x00;
    m_beacon_info[BINFO_LINE_BEACON_LOTKEY_IDX+1] = 0x00;
    m_beacon_info[BINFO_LINE_BEACON_LOTKEY_IDX+2] = 0x00;
    m_beacon_info[BINFO_LINE_BEACON_LOTKEY_IDX+3] = 0x00;
    m_beacon_info[BINFO_LINE_BEACON_LOTKEY_IDX+4] = 0x00;
    m_beacon_info[BINFO_LINE_BEACON_LOTKEY_IDX+5] = 0x00;
    m_beacon_info[BINFO_LINE_BEACON_LOTKEY_IDX+6] = 0x00;
    m_beacon_info[BINFO_LINE_BEACON_LOTKEY_IDX+7] = 0x00;
  }

  if (m_beacon_info[BINFO_BATTERY_LEVEL10_VALUE_IDX] == 0xFF) {
    m_beacon_info[BINFO_BATTERY_LEVEL10_VALUE_IDX] = BMS_DEFAULT_BATTERY_LEVEL10_VALUE;
  }
  
  if (m_beacon_info[BINFO_SYSCONFIG_FLAGS_IDX] == 0xFF) {
    m_beacon_info[BINFO_SYSCONFIG_FLAGS_IDX] = BMS_DEFAULT_SYSCONFIG_FLAGS;
  }

  if (m_beacon_info[BINFO_15SEC_TIMESTAMP_IDX] == 0xFF) {
    m_beacon_info[BINFO_15SEC_TIMESTAMP_IDX]   = 0x00;
    m_beacon_info[BINFO_15SEC_TIMESTAMP_IDX+1] = 0x00;
    m_beacon_info[BINFO_15SEC_TIMESTAMP_IDX+2] = 0x00;
    m_beacon_info[BINFO_15SEC_TIMESTAMP_IDX+3] = 0x00;
    m_beacon_info[BINFO_15SEC_TIMESTAMP_IDX+4] = 0x00;
    m_beacon_info[BINFO_15SEC_TIMESTAMP_IDX+5] = 0x00;
    m_beacon_info[BINFO_15SEC_TIMESTAMP_IDX+6] = 0x00;
    m_beacon_info[BINFO_15SEC_TIMESTAMP_IDX+7] = 0x00;
  }

  if (m_beacon_info[BINFO_TGSECB_TIMESTAMP_IDX] == 0xFF) {
    m_beacon_info[BINFO_TGSECB_TIMESTAMP_IDX]   = 0x00;
    m_beacon_info[BINFO_TGSECB_TIMESTAMP_IDX+1] = 0x00;
    m_beacon_info[BINFO_TGSECB_TIMESTAMP_IDX+2] = 0x00;
    m_beacon_info[BINFO_TGSECB_TIMESTAMP_IDX+3] = 0x00;
  }

  if (m_beacon_info[BINFO_TGSECB_SEC_KEY_IDX] == 0xFF) {
    m_beacon_info[BINFO_TGSECB_SEC_KEY_IDX]   = 0x00;
    m_beacon_info[BINFO_TGSECB_SEC_KEY_IDX+1] = 0x00;
    m_beacon_info[BINFO_TGSECB_SEC_KEY_IDX+2] = 0x00;
    m_beacon_info[BINFO_TGSECB_SEC_KEY_IDX+3] = 0x00;
    m_beacon_info[BINFO_TGSECB_SEC_KEY_IDX+4] = 0x00;
    m_beacon_info[BINFO_TGSECB_SEC_KEY_IDX+5] = 0x00;    
  }

  if (m_beacon_info[BINFO_TGSECB_ROTATED_BEACONID_IDX] == 0xFF) {
    m_beacon_info[BINFO_TGSECB_ROTATED_BEACONID_IDX]   = 0x00;
    m_beacon_info[BINFO_TGSECB_ROTATED_BEACONID_IDX+1] = 0x00;
    m_beacon_info[BINFO_TGSECB_ROTATED_BEACONID_IDX+2] = 0x00;
    m_beacon_info[BINFO_TGSECB_ROTATED_BEACONID_IDX+3] = 0x00;        
  }

  if (m_beacon_info[BINFO_SET_CURRENT_DATETIME_IDX] == 0xFF) {
    m_beacon_info[BINFO_SET_CURRENT_DATETIME_IDX]   = 0x00;
    m_beacon_info[BINFO_SET_CURRENT_DATETIME_IDX+1] = 0x00;
    m_beacon_info[BINFO_SET_CURRENT_DATETIME_IDX+2] = 0x00;
    m_beacon_info[BINFO_SET_CURRENT_DATETIME_IDX+3] = 0x00;
    m_beacon_info[BINFO_SET_CURRENT_DATETIME_IDX+4] = 0x00;
    m_beacon_info[BINFO_SET_CURRENT_DATETIME_IDX+5] = 0x00;
    m_beacon_info[BINFO_SET_CURRENT_DATETIME_IDX+6] = 0x00;
    m_beacon_info[BINFO_SET_CURRENT_DATETIME_IDX+7] = 0x00;
  }

  if (m_beacon_info[BINFO_ECO_MODE_START_TIME_IDX] == 0xFF) {
    m_beacon_info[BINFO_ECO_MODE_START_TIME_IDX]   = 0x00;
    m_beacon_info[BINFO_ECO_MODE_START_TIME_IDX+1] = 0x00;
  }

  if (m_beacon_info[BINFO_ECO_MODE_FINISH_TIME_IDX] == 0xFF) {
    m_beacon_info[BINFO_ECO_MODE_FINISH_TIME_IDX]   = 0x00;
    m_beacon_info[BINFO_ECO_MODE_FINISH_TIME_IDX+1] = 0x00;
  }

  if (m_beacon_info[BINFO_TIMESLOT_MODE_STATUS_IDX] == 0xFF) {
    m_beacon_info[BINFO_TIMESLOT_MODE_STATUS_IDX]   = 0x80;
  }

  if (m_beacon_info[BINFO_TIMESLOT_TXFRQ_VALUE_IDX] == 0xFF) {
    m_beacon_info[BINFO_TIMESLOT_TXFRQ_VALUE_IDX]   = 0x0F;
  }

  if (m_beacon_info[BINFO_TBM_TXFRQ_VALUE_IDX] == 0xFF) {
    m_beacon_info[BINFO_TBM_TXFRQ_VALUE_IDX]   = 0x02;
  }

  if (m_beacon_info[BINFO_BMS_BATTERY_MAX_CAPACITY_IDX] == 0xFF) {
#ifndef DEFAULT_HW_TYPE
    uint8_t max_battery_valtage[2] = {ENERGIZER_MAXIMUM_CAPACITY};
    m_Battery_Voltage_Max_Capacity = ENERGIZER_MAXIMUM_CAPACITY;
#else
    uint8_t max_battery_valtage[2] = {MAXBEACON_MAXIMUM_CAPACITY};
    m_Battery_Voltage_Max_Capacity = MAXBEACON_MAXIMUM_CAPACITY;
#endif
    m_beacon_info[BINFO_BMS_BATTERY_MAX_CAPACITY_IDX]   = max_battery_valtage[1];
    m_beacon_info[BINFO_BMS_BATTERY_MAX_CAPACITY_IDX+1] = max_battery_valtage[0];
  }

  if (m_beacon_info[BINFO_HARDWARE_TYPE_IDX] == 0xFF) {
#ifndef DEFAULT_HW_TYPE
    m_beacon_info[BINFO_HARDWARE_TYPE_IDX]   = HW_TYPE_TANGERINE_BEACON;
#else
    m_beacon_info[BINFO_HARDWARE_TYPE_IDX]   = HW_TYPE_MINEW_MAX_BEACON;
#endif
  }

}

void tb_manager_get_deviceid(void)
{
  uint32_t deviceID[2]; 
  deviceID[0] = NRF_FICR->DEVICEID[0]; 
  deviceID[1] = NRF_FICR->DEVICEID[1];
}
