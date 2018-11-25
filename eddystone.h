#ifndef _EDDYSTONE_H_
#define _EDDYSTONE_H_

#include "tb_global.h"
#include "advertising.h"

/*********************************************************************
   EDDTYSTONE Experimental Implementation 
   20160920: yama => 
**********************************************************************/
// Eddystone common data
#define APP_EDDYSTONE_UUID              0xFEAA                            /**< UUID for Eddystone beacons according
                                                                             to specification. */
#define APP_EDDYSTONE_RSSI              0xEE                              /**< 0xEE = -18 dB is the approximate sig
                                                                             nal strength at 0 m. */

// Eddystone UID data
#define APP_EDDYSTONE_UID_FRAME_TYPE    0x00                              /**< UID frame type is fixed at 0x00. */
#define APP_EDDYSTONE_UID_RFU           0x00, 0x00                        /**< Reserved for future use according to
                                                                             specification. */
#define APP_EDDYSTONE_UID_ID            0x01, 0x02, 0x03, 0x04, \
                                        0x05, 0x06                        /**< Mock values for 6-byte Eddystone UID
                                                                             ID instance.  */
#define APP_EDDYSTONE_UID_NAMESPACE     0xAA, 0xAA, 0xBB, 0xBB, \
                                        0xCC, 0xCC, 0xDD, 0xDD, \
                                        0xEE, 0xEE                        /**< Mock values for 10-byte Eddystone UID 
                                                                             ID namespace. */

// Eddystone URL data
#define APP_EDDYSTONE_URL_FRAME_TYPE    0x10                              /**< URL Frame type is fixed at 0x10. */

/**< URL Scheme Prefix
   0x00: "http://www" 
   0x01: "https://www."
   0x02: "http://"
   0x03: "https://"
*/
#define APP_EDDYSTONE_URL_SCHEME        0x03

#define APP_EDDYSTONE_URL_URL           0x67, 0x6f, 0x6f, 0x2e, 0x67, 0x6c, 0x2f, \
                                        0x51, 0x42, 0x4c, 0x39, 0x72, 0x57, \
                                        0x00, 0x00, 0x00, 0x00


// Eddystone TLM data
#define APP_EDDYSTONE_TLM_FRAME_TYPE    0x20                              /**< TLM frame type is fixed at 0x20. */
#define APP_EDDYSTONE_TLM_VERSION       0x00                              /**< TLM version might change in the futu
                                                                             re to accommodate other data according to specification. */
#define APP_EDDYSTONE_TLM_BATTERY       0x00, 0x00                        /**< Mock value. Battery voltage in 1 mV 
                                                                             per bit. */
#define APP_EDDYSTONE_TLM_TEMPERATURE   0x80, 0x00                        /**< Mock value. Temperature [C]. Signed 
                                                                             8.8 fixed-point notation. */
#define APP_EDDYSTONE_TLM_ADV_COUNT     0x00, 0x00, 0x00, 0x00            /**< Running count of advertisements of a
                                                                             ll types since power-up or reboot. */
#define APP_EDDYSTONE_TLM_SEC_COUNT     0x00, 0x00, 0x00, 0x00            /**< Running count in 0.1 s resolution si
                                                                             nce power-up or reboot. */
#define EDDYSTONE_URL_RSSI_CORRECTION   0x29                             /**< RSSI CORRECTION VALUE 1m -> 0m = 41dbm */


//
// public functions
//
void build_eddystone_tlm_data();
void build_eddystone_uid_data();
void build_eddystone_url_data();
void eddtystone_advertising_init(ble_advertising_mode_t eddystone_mode);
char* get_battery_percent_string();

#endif /* _EDDYSTONE_H_ */
