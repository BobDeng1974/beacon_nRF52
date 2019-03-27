#ifndef _ADVERTISING_H_
#define _ADVERTISING_H_

#include "tb_global.h"

//----------------------------------------------------------------------------
// Advertising mode
//----------------------------------------------------------------------------
typedef enum
{
  BLE_ADV_MODE_UNDEFINED       = 0, /**< Undefined : Terminator */
  BLE_ADV_MODE_BMS             = 1, /**< Tangerine Beacon Management Service */
  BLE_ADV_MODE_IBEACON         = 2, /**< iBeacon */
  BLE_ADV_MODE_EDDYSTONE_URL1  = 3, /**< EddtyStone-URL1 */
  BLE_ADV_MODE_EDDYSTONE_URL2  = 4, /**< EddtyStone-URL2 */
  BLE_ADV_MODE_EDDYSTONE_UID   = 5, /**< EddtyStone-UID */
  BLE_ADV_MODE_EDDYSTONE_TLM   = 6, /**< EddtyStone-TLM */
  BLE_ADV_MODE_LINE_IBEACON    = 7, /**< LINE iBeacon */
  BLE_ADV_MODE_LINE_PACKET     = 8,  /**< LINE Packet  */

  BLE_ADV_MODE_PAUSE           = 0xFF  /**< Pause  */
} ble_advertising_mode_t;

#define NUM_OF_BLE_ADVS 9　/** Number of supported BLE advertising types */

#include "ibeacon.h"
#include "eddystone.h"
#include "line.h"
#include "tbm_packet.h"

//----------------------------------------------------------------------------
// Parameter definitions
//----------------------------------------------------------------------------

// INTERVAL 
#define TIME_UNIT_MSEC                    1       /* time unit = msec */
#define TIME_UNIT_0625                    2       /* time unit = per 0.625msec */

#define APP_ADV_INTERVAL_0000             0x00A0  /* 0: 100ms ( 160 * 0.625ms) */
#define APP_ADV_INTERVAL_0000_MSEC        100
#define APP_ADV_INTERVAL_0000_UNITS       MSEC_TO_UNITS(APP_ADV_INTERVAL_0000_MSEC, UNIT_0_625_MS)
#define APP_ADV_INTERVAL_0000_USEC        10000

#define APP_ADV_INTERVAL_0001             0x0140  /* 1: 200ms ( 320 * 0.625ms) */
#define APP_ADV_INTERVAL_0001_MSEC        200
#define APP_ADV_INTERVAL_0001_UNITS       MSEC_TO_UNITS(APP_ADV_INTERVAL_0001_MSEC, UNIT_0_625_MS)
#define APP_ADV_INTERVAL_0001_USEC        20000

#define APP_ADV_INTERVAL_0010             0x0280  /* 2: 400ms ( 640 * 0.625ms) */
#define APP_ADV_INTERVAL_0010_MSEC        400
#define APP_ADV_INTERVAL_0010_UNITS       MSEC_TO_UNITS(APP_ADV_INTERVAL_0010_MSEC, UNIT_0_625_MS)
#define APP_ADV_INTERVAL_0010_USEC        40000

#define APP_ADV_INTERVAL_0011             0x0500  /* 3: 800ms (1280 * 0.625ms) */
#define APP_ADV_INTERVAL_0011_MSEC        800
#define APP_ADV_INTERVAL_0011_UNITS       MSEC_TO_UNITS(APP_ADV_INTERVAL_0011_MSEC, UNIT_0_625_MS)
#define APP_ADV_INTERVAL_0011_USEC        80000

#define APP_ADV_INTERVAL_0100             0x0A00  /* 4: 1600ms (2560 * 0.625ms) */
#define APP_ADV_INTERVAL_0100_MSEC        1600
#define APP_ADV_INTERVAL_0100_UNITS       MSEC_TO_UNITS(APP_ADV_INTERVAL_0100_MSEC, UNIT_0_625_MS)
#define APP_ADV_INTERVAL_0100_USEC        160000

#define APP_ADV_INTERVAL_0101             0x1400  /* 5: 3200ms (5120 * 0.625ms) */
#define APP_ADV_INTERVAL_0101_MSEC        3200
#define APP_ADV_INTERVAL_0101_UNITS       MSEC_TO_UNITS(APP_ADV_INTERVAL_0101_MSEC, UNIT_0_625_MS)
#define APP_ADV_INTERVAL_0101_USEC        320000

#ifdef FRQ_50MSEC
  #define APP_ADV_INTERVAL_0110             0x0050  /* 0: 50ms  ( 50 * 0.625ms) */
  #define APP_ADV_INTERVAL_0110_MSEC        50
  #define APP_ADV_INTERVAL_0110_UNITS       MSEC_TO_UNITS(APP_ADV_INTERVAL_0110_MSEC, UNIT_0_625_MS)
  #define APP_ADV_INTERVAL_0110_USEC        5000
#else
  #define APP_ADV_INTERVAL_0110             0x2800  /* 6: 6400ms (10240 * 0.625ms) */
  #define APP_ADV_INTERVAL_0110_MSEC        6400
  #define APP_ADV_INTERVAL_0110_UNITS       MSEC_TO_UNITS(APP_ADV_INTERVAL_0110_MSEC, UNIT_0_625_MS)
  #define APP_ADV_INTERVAL_0110_USEC        640000
#endif

#define APP_ADV_INTERVAL_0111             0x3E80  /* 7: 10000ms (16000 * 0.625ms) */
#define APP_ADV_INTERVAL_0111_MSEC        10000
#define APP_ADV_INTERVAL_0111_UNITS       MSEC_TO_UNITS(APP_ADV_INTERVAL_0111_MSEC, UNIT_0_625_MS)
#define APP_ADV_INTERVAL_0111_USEC        1000000

// following are added in v41009
#define APP_ADV_INTERVAL_1000             0x00C8  /* 8: 125ms (200 * 0.625ms) */
#define APP_ADV_INTERVAL_1000_MSEC        125
#define APP_ADV_INTERVAL_1000_UNITS       MSEC_TO_UNITS(APP_ADV_INTERVAL_1000_MSEC, UNIT_0_625_MS)
#define APP_ADV_INTERVAL_1000_USEC        12500

#define APP_ADV_INTERVAL_1001             0x00F0  /* 9: 150ms (240 * 0.625ms) */
#define APP_ADV_INTERVAL_1001_MSEC        150
#define APP_ADV_INTERVAL_1001_UNITS       MSEC_TO_UNITS(APP_ADV_INTERVAL_1001_MSEC, UNIT_0_625_MS)
#define APP_ADV_INTERVAL_1001_USEC        15000

#define APP_ADV_INTERVAL_1010             0x0118  /* 10: 175ms (280 * 0.625ms) */
#define APP_ADV_INTERVAL_1010_MSEC        175
#define APP_ADV_INTERVAL_1010_UNITS       MSEC_TO_UNITS(APP_ADV_INTERVAL_1010_MSEC, UNIT_0_625_MS)
#define APP_ADV_INTERVAL_1010_USEC        17500

#define APP_ADV_INTERVAL_1011             0x0168  /* 11: 225ms (360 * 0.625ms) */
#define APP_ADV_INTERVAL_1011_MSEC        225
#define APP_ADV_INTERVAL_1011_UNITS       MSEC_TO_UNITS(APP_ADV_INTERVAL_1100_MSEC, UNIT_0_625_MS)
#define APP_ADV_INTERVAL_1011_USEC        22500

#define APP_ADV_INTERVAL_1100             0x0190  /* 12: 250ms (400 * 0.625ms) */
#define APP_ADV_INTERVAL_1100_MSEC        250
#define APP_ADV_INTERVAL_1100_UNITS       MSEC_TO_UNITS(APP_ADV_INTERVAL_1100_MSEC, UNIT_0_625_MS)
#define APP_ADV_INTERVAL_1100_USEC        25000

#define APP_ADV_INTERVAL_1101             0x01B8  /* 13: 275ms (440 * 0.625ms) */
#define APP_ADV_INTERVAL_1101_MSEC        275
#define APP_ADV_INTERVAL_1101_UNITS       MSEC_TO_UNITS(APP_ADV_INTERVAL_1101_MSEC, UNIT_0_625_MS)
#define APP_ADV_INTERVAL_1101_USEC        27500

#define APP_ADV_INTERVAL_1110             0x01E0  /* 14: 300ms (480 * 0.625ms) */
#define APP_ADV_INTERVAL_1110_MSEC        300
#define APP_ADV_INTERVAL_1110_UNITS       MSEC_TO_UNITS(APP_ADV_INTERVAL_1110_MSEC, UNIT_0_625_MS)
#define APP_ADV_INTERVAL_1110_USEC        30000

#define APP_ADV_INTERVAL_1111             0x007A  /* 15: 76.25ms (122 * 0.625ms) */
#define APP_ADV_INTERVAL_1111_MSEC        76
#define APP_ADV_INTERVAL_1111_UNITS       MSEC_TO_UNITS(APP_ADV_INTERVAL_1111_MSEC, UNIT_0_625_MS)
#define APP_ADV_INTERVAL_1111_USEC        7625

#define ADV_INTERVAL_MARGIN               4       /* Advertising interval margin 4 * 0.625 = 3msec */
#define ADV_TIMEOUT_MARGIN                3       /* Advertising interval margin 3  msec */
#define APP_TIMER_PRESCALER               0       /**< Value of the RTC1 PRESCALER register. */


//
// Gloval variables
//

#ifdef ADV_SWITCH_TIMER_APP_TIMER
extern  app_timer_id_t                 g_adv_switch_timer_id;  /**< Advertising Switch Timer. */
#endif
extern  bool               m_adv_switch_timer_enabled;       /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */

extern  uint8_t            m_adv_switch_timer_start;
extern  uint8_t            m_adv_handle;

extern  ble_gap_adv_params_t  m_adv_params;                 /**< Parameters to be passed to the stack when starting advertising. */
extern  ble_gap_adv_data_t    m_ble_adv_data;

//
// public functions
//
ble_advertising_mode_t get_current_advmode();

void advertising_mode_reset();

void advertising_init(ble_advertising_mode_t mode);
void advertising_start(void);
void advertising_stop(void);
void advertising_restart(void);

void adv_switch(void);


void stop_adv_switch_timer();
void start_adv_switch_timer(uint8_t txFreq);

#ifdef ADV_SWITCH_TIMER_APP_TIMER
  void adv_switch_handler(TimerHandle_t xTimer);
#else
  void adv_switch_handler(void * p_context);
#endif

uint16_t get_beacon_frequency(uint8_t tx_freq, uint8_t unit);
uint32_t get_beacon_frequency2(uint8_t tx_freq, uint8_t unit);
void build_all_data();

#endif /* _ADVERTISING_H_ */
