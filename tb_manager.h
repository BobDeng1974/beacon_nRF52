#ifndef _TB_MANAGER_H_
#define _TB_MANAGER_H_

#include "tb_global.h"

/**
 * index and size of elements  in beacon_info
*/
#define  BINFO_IBEACON_UUID_IDX                0     // iBeacon UUID 
#define  BINFO_IBEACON_UUID_SIZ               16     //  -

#define  BINFO_MAJOR_VALUE_IDX                16     // iBeacon Major
#define  BINFO_MAJOR_VALUE_SIZ                 2     //  -

#define  BINFO_MINOR_VALUE_IDX                18     // iBeacon Minor
#define  BINFO_MINOR_VALUE_SIZ                 2     //  -

#define  BINFO_STATUS_VALUE_IDX               20     // Status 
#define  BINFO_STATUS_VALUE_SIZ                1     //  -

#define  BINFO_TXPWR_VALUE_IDX                21     // TxPower
#define  BINFO_TXPWR_VALUE_SIZ                 1     //  -

#define  BINFO_TXFRQ_VALUE_IDX                22     // TxFreq
#define  BINFO_TXFRQ_VALUE_SIZ                 1     //  -

#define  BINFO_BATTERY_VALUE_IDX              23     // Battery percent
#define  BINFO_BATTERY_VALUE_SIZ               1     //  -

#define  BINFO_SERIAL_ID_VALUE_IDX            24     // Serial ID
#define  BINFO_SERIAL_ID_VALUE_SIZ             4     //  -

#define  BINFO_SERVICE_ID_VALUE_IDX           28     // Service ID
#define  BINFO_SERVICE_ID_VALUE_SIZ            2     //  -

#define  BINFO_GEO_HASH_VALUE_IDX             30     // Geo Hash
#define  BINFO_GEO_HASH_VALUE_SIZ              8     //  -

#define  BINFO_VERSION_VALUE_IDX              40     // Version
#define  BINFO_VERSION_VALUE_SIZ               2     //  -

#define  BINFO_RSSI_VALUE_IDX                 42     // RSSI correction value
#define  BINFO_RSSI_VALUE_SIZ                  1     //  -

#define  ADVERTISE_SETTING_IDX                43     // Advertising setting
#define  ADVERTISE_SETTING_SIZ                 1     //  -

#define  BINFO_EDDYSTONE1_URL_LEN_IDX         44     // EDDYSTONE-URL #1 URL length
#define  BINFO_EDDYSTONE1_URL_LEN_SIZ          1     //  -

#define  BINFO_EDDYSTONE1_URL_URL_IDX         45     // EDDYSTONE-URL #1 URL 
#define  BINFO_EDDYSTONE1_URL_URL_SIZ         17     //  -

#define  BINFO_EDDYSTONE1_URL_SCH_IDX         62     // EDDYSTONE-URL #1 scheme
#define  BINFO_EDDYSTONE1_URL_SCH_SIZ          1     //  -

#define  BINFO_EDDYSTONE1_URL_BTC_IDX         63     // EDDYSTONE-URL #1 battery checking character
#define  BINFO_EDDYSTONE1_URL_BTC_SIZ          1     //  -

#define  BINFO_EDDYSTONE2_URL_LEN_IDX         64     // EDDYSTONE-URL #2 URL length
#define  BINFO_EDDYSTONE2_URL_LEN_SIZ          1     //  -

#define  BINFO_EDDYSTONE2_URL_URL_IDX         65     // EDDYSTONE-URL #2 URL 
#define  BINFO_EDDYSTONE2_URL_URL_SIZ         17     //  -

#define  BINFO_EDDYSTONE2_URL_SCH_IDX         82     // EDDYSTONE-URL #2 scheme
#define  BINFO_EDDYSTONE2_URL_SCH_SIZ          1     //  -

#define  BINFO_EDDYSTONE_URL_BTC_IDX          83     // EDDYSTONE-URL #2 battery checking character
#define  BINFO_EDDYSTONE_URL_BTC_SIZ           1     //  -
    
#define  BINFO_MODE_LIST_IDX                  84     // Advertising Mode list
#define  BINFO_MODE_LIST_SIZ                  12     //  -

#define  BINFO_TXPWR_FOR_MNG_IDX              96     // TxPower for management
#define  BINFO_TXPWR_FOR_MNG_SIZ               1     //  -

#define  BINFO_EDDYSTONE_UID_ID_IDX           97     // EDDYSTONE-UID instanceID
#define  BINFO_EDDYSTONE_UID_ID_SIZ            6     //  -

#define  BINFO_EDDYSTONE_UID_NAMESPACE_IDX   103     // EDDYSTONE-UID Namaspace
#define  BINFO_EDDYSTONE_UID_NAMESPACE_SIZ    10     //  -

#define  BINFO_LINE_BEACON_HWID_IDX          113     // LINE Beacon HWID allocated bv LINE corp
#define  BINFO_LINE_BEACON_HWID_SIZ            5     // LINE Beacon HWIN allocated bv LINE corp

#define  BINFO_LINE_BEACON_VENDOR_KEY_IDX    118     // LINE Beacon VendorKey allocated bv LINE corp
#define  BINFO_LINE_BEACON_VENDOR_KEY_SIZ      4     // LINE Beacon VendorKey allocated bv LINE corp

#define  BINFO_LINE_BEACON_LOTKEY_IDX        122     // LINE Beacon LotKey allocated bv Tangerine
#define  BINFO_LINE_BEACON_LOTKEY_SIZ          8     // LINE Beacon LotKey allocated bv Tangerine

#define  BINFO_BATTERY_LEVEL10_VALUE_IDX     130     // l=1 : s=130 : Battery Level divided by 10
#define  BINFO_BATTERY_LEVEL10_VALUE_SIZ       1     // -

#define  BINFO_SYSCONFIG_FLAGS_IDX           131     // l=1 : s=131 : system configuration flag
#define  BINFO_SYSCONFIG_FLAGS_SIZ             1     // -

#define  BINFO_15SEC_TIMESTAMP_IDX           132     // l=8 : s=132 : 15sec timestamp counter
#define  BINFO_15SEC_TIMESTAMP_SIZ             8     // -

#define BINFO_TGSECB_TIMESTAMP_IDX           140     // l=4 : s=140 : tg secure timestamp
#define BINFO_TGSECB_TIMESTAMP_SIZ             4     // -

#define BINFO_TGSECB_SEC_KEY_IDX             144     // l=6 : s=144 : tg secure secure key
#define BINFO_TGSECB_SEC_KEY_SIZ               6     // -

#define BINFO_TGSECB_ROTATED_BEACONID_IDX    150     // l=4 : s=150 : tg secure rotated beaconID
#define BINFO_TGSECB_ROTATED_BEACONID_SIZ      4     // -

#define BINFO_SET_CURRENT_DATETIME_IDX       154     // l=8 : s=154 : Set Current date/time
#define BINFO_SET_CURRENT_DATETIME_SIZ         7     // -

#define BINFO_ECO_MODE_START_TIME_IDX        162     // l=2 : s=162 : Eco mode start time
#define BINFO_ECO_MODE_START_TIME_SIZ          2     //  -

#define BINFO_ECO_MODE_FINISH_TIME_IDX       164     // l=2 : s=164 : Eco mode finish time
#define BINFO_ECO_MODE_FINISH_TIME_SIZ         2     //  -

#define BINFO_CURRENT_DATETIME_IDX           166     // l=8 : s=166 : Current date/time
#define BINFO_CURRENT_DATETIME_SIZ             7     // -

#define BINFO_TIMESLOT_MODE_STATUS_IDX       174     // l=1 : s=174 : TIMESLOT mode status
#define BINFO_TIMESLOT_MODE_STATUS_SIZ         1     // -

#define BINFO_TIMESLOT_TXFRQ_VALUE_IDX       176     // l=12 : s=176 : Timeslot length
#define BINFO_TIMESLOT_TXFRQ_VALUE_SIZ        1     // -

#define BINFO_TBM_TXFRQ_VALUE_IDX            178     // l=1 : s=178 : Timeslot advertising distance
#define BINFO_TBM_TXFRQ_VALUE_SIZ              1     // -

#define BINFO_BMS_BATTERY_MAX_CAPACITY_IDX   180     // l=2 : s=180 : Battery Maximum capacity
#define BINFO_BMS_BATTERY_MAX_CAPACITY_SIZ     2     // -

#define BINFO_HARDWARE_TYPE_IDX              182     // l=1 : s=182 : Hardware Type
#define BINFO_HARDWARE_TYPE_SIZ                2     // -

#define MAX_ADV_MODE_LIST                     12     // Advertising Mode List Max Index


#define BMS_BEACON_INFO_LENGTH                  190


/**
 * Public functions
*/
void     tb_manager_pstorage_init(void);
uint32_t tb_manager_init();
uint32_t tb_manager_settings_load(void);
uint32_t tb_manager_settings_store(void);
uint8_t* ble_bms_get_beacon_info(void);

uint8_t  ble_ibeacon_enablep();
uint8_t  ble_eddystone_url1_enablep();
uint8_t  ble_eddystone_url2_enablep();
uint8_t  ble_eddystone_uid_enablep();
uint8_t  ble_eddystone_tlm_enablep();
uint8_t  ble_line_beacon_enablep();
uint8_t ble_tgsec_ibeacon_enablep();

void     ble_bms_reset_beacon_info();
void     tb_manager_get_deviceid(void);

uint8_t   ble_batteryp();
uint8_t   ble_tibeacon_pwinfop();

void ble_bms_set_default_value_to_beacon_info();
uint32_t tb_manager_reset(void);

#endif /* _TB_MANAGER_H_ */
