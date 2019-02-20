#ifndef _BEACON_DEVICE_H_
#define _BEACON_DEVICE_H_

#define BLE_BMS_RAPINAVI2_H__

#include "tb_global.h"
#include "nrf_soc.h"



#define APP_GPIOTE_MAX_USERS  1           /**< Maximum number of users of the GPIOTE handler. */

/*
  global variables 
*/
#define TX_POWER_LEVEL  (0)               /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */
#ifdef _BEACON_DEVICE_MAIN_CODE_
uint8_t ble_bms_rssi[9] = {
  -61 + 256,     // 0dbm : TxPower(0) iBeacon OFF
  -93 + 256,    // -30dbm : TxPower(1) -30
  -80 + 256,     // -20dbm : TxPower(2) -20
  -78 + 256,     // -16dbm : TxPower(3) -16
  -73 + 256,     // -12dbm : TxPower(4) -12
  -69 + 256,     // -8dbm  : TxPower(5) -8
  -67 + 256,     // -4dbm  : TxPower(6) -4
  -61 + 256,     //  0dbm  : TxPower(7) 0
  -58 + 256     //  4dbm  : TxPower(8) 4
};

#else 

extern uint8_t ble_bms_rssi[9];

#endif /* _BEACON_DEVICE_MAIN_CODE_ */

// Debug
#define CHECK_SIGNAL_H()  nrf_gpio_pin_set(DEBUG_PIN)
#define CHECK_SIGNAL_L()  nrf_gpio_pin_clear(DEBUG_PIN)

// LED Configuration
#define BATTERY_CHECKING_ENABLED // 
#define ECOMODE_CHECKING_ENABLED // 
#define LED_ENABLED

#define BEACON_LED     LED_G       // LED GPIO port
#define BEACON_LED_ERR LED_R       // ERR LED GPIO port

#define BEACON_LED_ACTIVE_HIGH
#define LED_ON  "1"
#define LED_OFF "0"

static uint8_t m_Blink_LED_count;
static uint8_t m_Blink_Error_LED_count;
static uint8_t m_Execute_led_flash_type1;

/**
 * GPIO and LED 
 */
void gpiote_init(void);                       // GPIO initialize
void gpiote_init_hw_type(uint8_t hw_type);    // GPIO hardware typw initialize
void execute_led(const char *param);          // turn on/off LED
void led_flash_type1();                       // flash led type1
void execute_error_led(const char *param);    // turn on/off RED LED
void blink_led(uint8_t count);                // Blink LED
void blink_error_led(uint8_t count);          // Blink Error LED
void blink_pending_led(uint8_t c, uint8_t t); // Blink Pending LED
void execute_pending_led(const char *param);  // Blink Pending LED

/**
 * rssi / tx power
 */
int8_t get_adjusted_rssi(uint8_t tx_power);
int8_t get_tx_power_level(uint8_t tx_power);

/**
 * battery level
 */
extern  uint16_t m_Energizer_Max_Capacity;

void battery_init(void);
uint16_t get_battery_level(void);

/**
 * Retrieve current battery level
 */
uint16_t battery_level_get(void);
uint16_t battery_level_get2(void);

/**
 * Convert battery level(mv) to percentage
 */
uint8_t battery_level_to_percent(const uint16_t mvolts);

/**
 * RTC PCF8563
 */

#define SCL_PIN_			7
#define SDA_PIN_			6	
 
#define SDA_H()	nrf_gpio_pin_set(SDA_PIN_)
#define SCL_H() nrf_gpio_pin_set(SCL_PIN_)
#define SDA_L() nrf_gpio_pin_clear(SDA_PIN_)
#define SCL_L() nrf_gpio_pin_clear(SCL_PIN_)

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t days;
	uint8_t weekdays;
	uint8_t months;
	uint8_t years;
} PRE_TIME;

extern  PRE_TIME m_pre_time;

uint8_t pcf8563_init(uint8_t scl_pin, uint8_t sda_pin);
uint8_t pcf8563_write(void);
uint8_t pcf8563_read(void);


#endif /* _BEACON_DEVICE_H_ */