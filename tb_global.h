#ifndef _TB_GLOBAL_H_
#define _TB_GLOBAL_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>
#include <time.h>

#include "system_nrf52.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advertising.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_types.h"
#include "ble_gap.h"
#include "nrf_ble_gatt.h"
#include "boards.h"
#include "app_timer.h"
#include "app_gpiote.h"
#include "nrf_delay.h"

#include "nrf_ble_qwr.h"
#include "fds.h"
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "nrf_power.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_bootloader_info.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_saadc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "ble_dfu.h"
#include "nrf_dfu_ble.h"

#include "tb_manager.h"
#include "ble_bms.h"
#include "beacon_device.h"
#include "advertising.h"

#include "tracetask.h"

//#define ADV_SWITCH_TIMER_APP_TIMER
//#define FREERTOS_SWITCH
#define TIMESLOT_DEBUG
#define RGB_LED
#define PCA10040

#include "custom_board.h"

#ifdef   FREERTOS_SWITCH
  #include "nrf_sdh_freertos.h"
  #include "FreeRTOS.h"
  #include "task.h"
  #include "timers.h"
  #include "semphr.h"
#endif

#define bcd2bin(x)  const_bcd2bin(x)
#define bin2bcd(x)  const_bin2bcd(x)

#define const_bcd2bin(x)	(((x) & 0x0f) + ((x) >> 4) * 10)
#define const_bin2bcd(x)	((((x) / 10) << 4) + (x) % 10)

//----------------------------------------------------------------------------
// version number
//----------------------------------------------------------------------------
//#define APP_FIRMWARE_VERSION_VALUE    0xA0, 0x20 /* v40992 */
//#define APP_FIRMWARE_VERSION_VALUE    0xA0, 0x30 /* v41008 EDDYSTONE-URL support */
//#define APP_FIRMWARE_VERSION_VALUE    0xA0, 0x31 /* v41009 update from v41008 */
//#define APP_FIRMWARE_VERSION_VALUE    0xA0, 0x40 /* v41024 EDDYSTONE-UID, TLM support */
//#define APP_FIRMWARE_VERSION_VALUE    0xA0, 0x50 /* v41040 refactoring version */
//#define APP_FIRMWARE_VERSION_VALUE    0xA0, 0x60 /* v41056 LINE support version */
//#define APP_FIRMWARE_VERSION_VALUE    0xA0, 0x70 /* v41072 Tangerine Secure Beacon v1.0 support */
#define APP_FIRMWARE_VERSION_VALUE    0xA4, 0x89 /* v42121 nRF52 Tangerine Secure Beacon v1.1 support */

//----------------------------------------------------------------------------
// device name and company identifier
//----------------------------------------------------------------------------
#define DEVICE_NAME                     "TgReF"   /**< Name of device. Will be included in the advertising data. */
#define TANGERINE_COMPANY_IDENTIFIER    0x014E     /*< Company identifier for Tangerine Inc. as per www.bluetooth.org. */

//----------------------------------------------------------------------------
// 
//----------------------------------------------------------------------------
#define OSTIMER_WAIT_FOR_QUEUE          2                                  /**< Number of ticks to wait for the timer queue to be ready */
#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                300                                /**< The advertising interval (in units of 0.625 ms). This value corresponds to 187.5 ms. */
#define APP_ADV_DURATION                18000                              /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                /**< The advertising timeout in units of seconds. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define ADC_SAMPLES_IN_BUFFER           5
#define ENERGIZER_MAXIMUM_CAPACITY      46800                             // Energizer Maximum capacity

#define ble_bms_get_timeslot_status()   m_timeslot_mode

//----------------------------------------------------------------------------
//  global variables 
//----------------------------------------------------------------------------
extern  uint8_t           m_tbm_scan_mode;
extern  uint8_t           m_timeslot_mode;
extern  uint8_t           m_advertising_packet_type;
extern  uint8_t           m_eco_adv_stop;
extern  ble_gap_addr_t    m_device_addr;          // 48-bit address, LSB format
extern  ble_advertising_t m_advertising;          // Advertising module instance. BLE_ADVERTISING_DEF(m_advertising); */
extern  nrf_saadc_value_t m_buffer_pool[ADC_SAMPLES_IN_BUFFER];                                        
extern  uint8_t           m_adc_status;
extern  uint8_t           m_bTbmRequest;          // Tangerine Beacon Management Packet
extern  uint8_t           m_bTbmRequestCounter;   // Tangerine Beacon Management Packet
extern  uint8_t           m_bPending;

extern  uint8_t           g_is_startup;           // startup status
extern  uint8_t           g_connected;            // connected or not connected
extern  uint16_t          g_conn_handle;          // 
extern  uint8_t           g_ibeacon_mode;
extern  ble_bms_t         g_bms;                  // Structure to identify the Beacon Management Service. */
extern  uint8_t           g_startup_stage;        // startup status

//-----------------------------------------------------------------------------
// transable uint* type : uint* <-> uint8_t type array
//----------------------------------------------------------------------------
// union type for int32, uint16
typedef union {
  uint32_t value;
  uint8_t array[4];
} uint32_union_t;

typedef union {
  uint16_t value;
  uint8_t array[2];
} uint16_union_t;

typedef union {
  uint64_t value;
  uint8_t array[8];
} uint64_union_t;

typedef union {
  unsigned short int wTime;
  struct {
    unsigned char Hours;
    unsigned char Minutes;
  } dec;
} dectime_union_t;

//-----------------------------------------------------------------------------
// counters
//----------------------------------------------------------------------------
extern  uint32_union_t m_adv_count;           /**< advertising counter */
extern  uint32_union_t m_sec_count_100ms;     /**< 100msec counter */
extern  uint16_union_t m_battery_charge;      /**< current battery charge */
extern  uint64_union_t m_line_timestamp;      /**< 64bit timestamp for LINE Beacon */
extern  uint32_union_t m_tgsec_timestamp;     /**< 32bit timestamp for Tangerine Secure iBeacon */

extern  dectime_union_t m_eco_start_time;     /**< 16bit decmal eco mode start time */
extern  dectime_union_t m_eco_finish_time;    /**< 16bit decmal eco mode finish time */

#endif /*  _TB_GLOBAL_H_ */

