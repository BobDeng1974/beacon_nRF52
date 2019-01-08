#ifndef _IBEACON_H_
#define _IBEACON_H_

#include "tb_global.h"
#include "advertising.h"

#if defined(IBEACON_WITH_BATT_INFO)
#define APP_CLBEACON_INFO_LENGTH      0x18          /**< Total length of information advertised by the iBeacon. */
#else
#define APP_CLBEACON_INFO_LENGTH      0x17          /**< Total length of information advertised by the iBeacon. */
#endif 

#define APP_DEVICE_TYPE               0x02, 0x15    /**< 0x02, 0x15 device type on iBeacon. */
#define APP_MEASURED_RSSI             0xAD          /**< The iBeacon's measured RSSI at 1 meter distance in dBm. 0xC5=-59 C3 AD=-81*/
#define APP_COMPANY_IDENTIFIER        0x004C        /**< Company identifier for Apple Inc. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE               0x00, 0x00    /**< Major value used to identify iBeacons. */
#define APP_MINOR_VALUE               0x00, 0x00    /**< Minor value used to identify iBeacons. */
#define APP_IBEACON_UUID              0x01, 0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89, 0x9A, 0xAB, 0xBC, 0xCD, 0xDE, 0xEF, 0xF0

void build_ibeacon_data(void);
void ibeacon_advertising_init();
uint8_t * get_ibeacon_advertising_data(void);

void set_ibeacon_packet(void);
uint8_t * get_ibeacon_packet(void);


#endif /* _IBEACON_H_ */
