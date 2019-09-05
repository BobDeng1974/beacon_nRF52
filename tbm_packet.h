#ifndef _TBM_PACKET_H_
#define _TBM_PACKET_H_

#include "tb_global.h"
#include "ble_bms.h"
#include "advertising.h"

//#define APP_BMS_INFO_LENGTH           0x18                              /**< Total length of information advertised by BMS. */
#define APP_BMS_INFO_LENGTH             0x17                              /**< Total length of information advertised by BMS. */
#define APP_BMS_STATUS_INDEX            21
#define APP_BMS_TXPWR_INDEX             22
#define APP_BMS_TXFRQ_INDEX             23
#define APP_BMS_BATTERY_INDEX           24

extern  uint8_t m_fcm;

void build_bms_data(void);
void bms_advertising_init(ble_bms_t m_bms);
uint8_t * get_bms_advertising_data(void);
uint8_t * get_bms_info(void);
uint8_t * get_tbm_packet(void);
void tx_power_set(void);

#endif /*  _TBM_PACKET_H_ */
