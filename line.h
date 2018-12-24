#ifndef _LINE_BEACON_H_
#define _LINE_BEACON_H_

#include "tb_global.h"
#include "advertising.h"

// LINE iBeacon frame
#define APP_LINE_IBEACON_INFO_LENGTH          0x17          /**< Total length of information advertised by the iBeacon. */
#define APP_LINE_IBEACON_ADV_DATA_LENGTH      0x15          /**< Length of manufacturer specific data in the advertisement. */
#define APP_LINE_IBEACON_DEVICE_TYPE          0x02          /**< 0x02 refers to iBeacon. */
#define APP_LINE_IBEACON_MEASURED_RSSI        0x00          /**< The iBeacon's measured RSSI at 1 meter distance in dBm. 0xC5=-59 C3 AD=-81*/
#define APP_LINE_IBEACON_COMPANY_IDENTIFIER   0x004C        /**< Company identifier for Apple Inc. as per www.bluetooth.org. */
#define APP_LINE_IBEACON_MAJOR_VALUE          0x4C, 0x49    /**< Major value used to identify iBeacons. */
#define APP_LINE_IBEACON_MINOR_VALUE          0x4E, 0x45    /**< Minor value used to identify iBeacons. */
#define APP_LINE_IBEACON_UUID                 0xD0, 0xD2, 0xCE, 0x24, 0x9E, 0xFC, 0x11, 0xE5, 0x82, 0xC4, 0x1C, 0x6A, 0x7A, 0x17, 0xEF, 0x38

// LINE Beacon Packet service frame
//#define APP_LINE_BEACON_SERVICE_FRAME_LENGTH  0x0D
#define APP_LINE_BEACON_SERVICE_FRAME_LENGTH  0x0E
#define LINE_BEACON_SERVICE_UUID              0x6F, 0xFE
#define LINE_BEACON_TYPE                      0x02
#define LINE_BEACON_HWID                      0x01, 0x02, 0x03, 0x04, 0x05
#define LINE_BEACON_TXPW                      0x00
#define LINE_BEACON_SECMSG                    0x01, 0x02, 0x03, 0x04,0x05, 0x06, 0x07
#define LINE_HWID_LENGTH                       5
#define LINE_VENDORKEY_LENGTH                  4
#define LINE_LOTKEY_LENGTH                     8
#define LINE_SECMSG_ORIGIN_LENGTH             26
#define LINE_SECMSG_LENGTH                     7

void build_line_ibeacon_data(void);
void line_ibeacon_advertising_init();

void build_line_beacon_packet_servicedata(void);
void line_beacon_packet_advertising_init();

void set_line_ibeacon_packet(void);
uint8_t * get_line_ibeacon_packet(void);

void set_line_beacon_packet(void);
uint8_t * get_line_beacon_packet(void);

void line_sha256_init(void);

#endif /* _LINE_BEACON_H_ */
