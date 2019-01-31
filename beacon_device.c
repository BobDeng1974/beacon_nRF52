#define _BEACON_DEVICE_MAIN_CODE_

#include "beacon_device.h"


/* LED */
static uint8_t m_Blink_LED_count;
static uint8_t m_Blink_Error_LED_count;
static uint8_t m_Execute_led_flash_type1;

/* Battery */
uint16_t m_Energizer_Max_Capacity = ENERGIZER_MAXIMUM_CAPACITY;

/* RTC PCF8563 */
#define RTC_SLAVE_ADDR        0x51
#define RTC_SECONDS_ADDR      0x02
#define RTC_TIMEDATE_LENGTH   0x07

PRE_TIME m_pre_time;

/* TWI instance ID. */
#define TWI_INSTANCE_ID 0
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static bool m_xfer_done;

/**@brief Function for initializing the GPIOTE handler module.
 */

#ifdef PCA10040
void execute_led(const char *param)
{
  if (param[0] == '1') {
    nrf_gpio_pin_clear(LED_G);
    nrf_gpio_pin_set(LED_R);
  }
  else {
    nrf_gpio_pin_set(LED_G);
    nrf_gpio_pin_set(LED_R);
  }
}

void execute_error_led(const char *param)
{
  if (param[0] == '1') {
    nrf_gpio_pin_clear(LED_R);
  }
  else {
    nrf_gpio_pin_set(LED_R);
  }
}

void execute_pending_led(const char *param)
{
  if (param[0] == '1') {
    nrf_gpio_pin_clear(LED_B);
  }
  else {
    nrf_gpio_pin_set(LED_3);
  }
}
#else
  #ifndef RGB_LED
void execute_led(const char *param)
{
  if (param[0] == '0') {
    nrf_gpio_pin_clear(LED_G);
    nrf_gpio_pin_clear(LED_R);
  }
  else {
    nrf_gpio_pin_set(LED_G);
    nrf_gpio_pin_clear(LED_R);
  }
}

void execute_error_led(const char *param)
{
  if (param[0] == '0') {
    nrf_gpio_pin_clear(LED_R);
  }
  else {
    nrf_gpio_pin_set(LED_R);
  }
}

void execute_pending_led(const char *param)
{
  if (param[0] == '0') {
    nrf_gpio_pin_clear(LED_G);
    nrf_gpio_pin_clear(LED_R);
  }
  else {
    nrf_gpio_pin_set(LED_G);
    nrf_gpio_pin_set(LED_R);
  }
}
  #else
void execute_led(const char *param)
{
  if (param[0] == '1') {
    nrf_gpio_pin_clear(LED_G);
    nrf_gpio_pin_clear(LED_R);
    nrf_gpio_pin_clear(LED_B);
  }
  else {
    nrf_gpio_pin_set(LED_G);
    nrf_gpio_pin_set(LED_R);
    nrf_gpio_pin_set(LED_B);
  }
}

void execute_error_led(const char *param)
{
  if (param[0] == '1') {
    nrf_gpio_pin_set(LED_G);
    nrf_gpio_pin_clear(LED_R);
    nrf_gpio_pin_set(LED_B);
  }
  else {
    nrf_gpio_pin_set(LED_G);
    nrf_gpio_pin_set(LED_R);
    nrf_gpio_pin_set(LED_B);
  }
}

void execute_pending_led(const char *param)
{
  if (param[0] == '1') {
    nrf_gpio_pin_set(LED_G);
    nrf_gpio_pin_set(LED_R);
    nrf_gpio_pin_clear(LED_B);
  }
  else {
    nrf_gpio_pin_set(LED_G);
    nrf_gpio_pin_set(LED_R);
    nrf_gpio_pin_set(LED_B);
  }
}
  #endif
#endif

void blink_led(uint8_t count)
{
  for(uint8_t i = 0; i < count; i++) 
  {
    execute_led(LED_ON);
    nrf_delay_ms(200);
    //vTaskDelay(200);

    execute_led(LED_OFF);
    nrf_delay_ms(200);
    //vTaskDelay(200);
  }
}

void blink_error_led(uint8_t count)
{
  for(uint8_t i = 0; i < count; i++) 
  {
    execute_error_led(LED_ON);
    nrf_delay_ms(200);

    execute_error_led(LED_OFF);
    nrf_delay_ms(200);
  }
}

void led_flash_type1()
{
  execute_led(LED_ON);
  nrf_delay_ms(1500);

  execute_led(LED_OFF);
  nrf_delay_ms(1000);

  execute_led(LED_ON);
  nrf_delay_ms(1500);

  execute_led(LED_OFF);
  nrf_delay_ms(1000);

  execute_led(LED_ON);
  nrf_delay_ms(1500);

  execute_led(LED_OFF);
  nrf_delay_ms(1000);

  execute_led(LED_ON);
  nrf_delay_ms(1500);

  execute_led(LED_OFF);
  nrf_delay_ms(1000);
}

void gpiote_init(void)
{
  //nrf_gpio_cfg_output(DEBUG_PIN);
  //nrf_gpio_pin_set(DEBUG_PIN);
  //nrf_gpio_cfg_output(DEBUG_PIN2);
  //nrf_gpio_pin_set(DEBUG_PIN2);

  nrf_gpio_cfg_output(LED_G);
  nrf_gpio_cfg_output(LED_R);
  nrf_gpio_cfg_output(LED_B);
  nrf_gpio_pin_clear(LED_G);
  nrf_gpio_pin_set(LED_R);
  nrf_gpio_pin_set(LED_B);
}

void battery_init(void)
{
  ret_code_t err_code;
 
  nrf_saadc_channel_config_t adc_channel_conf = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(BAT_V);

  err_code = nrf_drv_saadc_channel_init(0, &adc_channel_conf);
  APP_ERROR_CHECK(err_code);
}

uint16_t get_battery_level()
{
  uint16_t blevel = 0;
  uint16_t bl_max = 0;
  uint16_t bl_min = 10000;

  for (int i = 0; i < 10; i++) {
    uint16_t bl_tmp = battery_level_get2(); 

    if (bl_tmp > bl_max) {
      // 最大値更新
      bl_max = bl_tmp;
    }
    if (bl_tmp < bl_min) {
      // 最小値更新
      bl_min = bl_tmp;
    }

    blevel += bl_tmp;
    
    nrf_delay_us(1);
  }

  // 最大値と最小値を排除
  blevel -= bl_max;
  blevel -= bl_min;

  // 平均値を計算
  blevel = (blevel / 8);

  return blevel;
}

#define MAXADBF 512
uint32_t m_adbuffer[MAXADBF];
uint32_t m_adbix;

#define MAX_BATTERY_ADC_SAMPLE  10
uint16_t battery_level_get2(void)
{
  nrf_saadc_value_t adc_result;
  uint32_t err_code;
  uint32_t adc_total = 0;
  uint32_t adc_sample = 0;

  for (int ix=0; ix < MAX_BATTERY_ADC_SAMPLE; ix++)
  {   
    err_code = nrf_drv_saadc_sample_convert(0, &adc_result);
    APP_ERROR_CHECK(err_code);
    adc_total += (uint32_t)adc_result;
    //nrf_delay_ms(50);
  }

  adc_sample = (uint32_t)(adc_total / MAX_BATTERY_ADC_SAMPLE);    
  if ( m_adbix < MAXADBF )
  {
    m_adbuffer[m_adbix] = adc_sample;
    m_adbix++;
  }
  return adc_sample;

  uint16_t vbg_in_mv = 1200;
  uint16_t adc_max = 255; // 10bit=1023, 8bit=255
  uint16_t vbat_current_in_mv = ((adc_sample) * 3 * vbg_in_mv) / adc_max;

  return vbat_current_in_mv;
}

uint16_t battery_level_get()
{
  ret_code_t err_code;

  //saadc_init();

  //Event handler is called immediately after conversion is finished.
  //err_code = nrf_drv_saadc_sample(); // Check error
  //APP_ERROR_CHECK(err_code);

  m_adc_status = true;
  while (m_adc_status == true) {};
  //nrf_drv_saadc_uninit();

  uint16_t adc_result = 0;
  for(int i=0; i < ADC_SAMPLES_IN_BUFFER; i++) adc_result += m_buffer_pool[i];
  
  uint16_t vbg_in_mv = 1200;
  uint16_t adc_max = 255; // 10bit=1023, 8bit=255
  uint16_t vbat_current_in_mv = ((adc_result/ADC_SAMPLES_IN_BUFFER) * 3 * vbg_in_mv) / adc_max;

  return vbat_current_in_mv;
}

uint8_t battery_level_to_percent(const uint16_t mvolts)
{
    float fMaxCapacity = (float)m_Energizer_Max_Capacity;

    fMaxCapacity = ((float)mvolts / fMaxCapacity) * 100.0;
    uint8_t battery_level = (uint8_t)fMaxCapacity;
    if ( battery_level > 100 ) battery_level = 100;
    return battery_level;
}

int8_t get_adjusted_rssi(uint8_t tx_power) 
{
  // Tx Power LevelによってRSSI値を変更する
  if (tx_power < 0 || tx_power >= 9) {
    tx_power = 0;
  }

  //return ble_bms_rssi[tx_power];
  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  return ble_bms_rssi[tx_power] + _beacon_info[BINFO_RSSI_VALUE_IDX];
}

int8_t get_tx_power_level(uint8_t tx_power) 
{
  // Beacon Tx Power Level
  switch( tx_power ) {
  case 0:
    // 0の場合、iBeaconパケットのみ送信しないようにする
    // return -40;
    return 0;
  case 1:
    return RADIO_TXPOWER_TXPOWER_Neg40dBm;  /*!< -40 dBm */
  case 2:
    return RADIO_TXPOWER_TXPOWER_Neg20dBm;  /*!< -20 dBm */
  case 3:
    return RADIO_TXPOWER_TXPOWER_Neg16dBm;  /*!< -16 dBm */
  case 4:
    return RADIO_TXPOWER_TXPOWER_Neg12dBm;  /*!< -12 dBm */
  case 5:
    return RADIO_TXPOWER_TXPOWER_Neg8dBm;   /*!< -8 dBm */
  case 6:
    return RADIO_TXPOWER_TXPOWER_Neg4dBm;   /*!< -4 dBm */
  case 7:
    return RADIO_TXPOWER_TXPOWER_0dBm;      /*!< 0 dBm */
  case 8:
    return RADIO_TXPOWER_TXPOWER_Pos4dBm;   /*!< +4 dBm */
  case 9:
    return RADIO_TXPOWER_TXPOWER_Pos3dBm;   /*!< +3 dBm */
  }
  return TX_POWER_LEVEL;
}


void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context)
{
  switch (p_event->type)
  {
  case NRF_DRV_TWI_EVT_DONE:
    m_xfer_done = true;
    break;
  case NRF_DRV_TWI_EVT_ADDRESS_NACK:
    m_xfer_done = 0xFF;
    break;
  case NRF_DRV_TWI_EVT_DATA_NACK:
    m_xfer_done = 0xFF;
    break;
  default:
    break;
  }
}

uint8_t PCF8563_read_regs(uint8_t number, uint8_t length, uint8_t *values)
{
  m_xfer_done = false;
  ret_code_t err_code = nrf_drv_twi_tx(&m_twi, RTC_SLAVE_ADDR, &number, 1, true);
  APP_ERROR_CHECK(err_code);
  while (m_xfer_done == false) {
    if (m_xfer_done == 0xFF) return false;
  }

  m_xfer_done = false;
  err_code = nrf_drv_twi_rx(&m_twi, RTC_SLAVE_ADDR, values, length);
  APP_ERROR_CHECK(err_code);
  while (m_xfer_done == false) {
    if (m_xfer_done == 0xFF) return false;
  }
  return true;
}

uint8_t PCF8563_write_regs(uint8_t length, uint8_t *values)
{
  m_xfer_done = false;
  ret_code_t err_code = nrf_drv_twi_tx(&m_twi, RTC_SLAVE_ADDR, values, length, false);
  APP_ERROR_CHECK(err_code);
  while (m_xfer_done == false) {
    if (m_xfer_done == 0xFF) return false;
  }
  return true;
}

uint8_t pcf8563_write(void)
{
  uint8_t data[RTC_TIMEDATE_LENGTH+1];

  data[0] = RTC_SECONDS_ADDR;
  data[1] = m_pre_time.seconds;
  data[2] = m_pre_time.minutes;
  data[3] = m_pre_time.hours;
  data[4] = m_pre_time.days;
  data[5] = m_pre_time.weekdays;
  data[6] = m_pre_time.months;
  data[7] = m_pre_time.years;
  return PCF8563_write_regs(RTC_TIMEDATE_LENGTH+1, data);
}

uint8_t pcf8563_read(void)
{
  uint8_t data[RTC_TIMEDATE_LENGTH];
  uint8_t result = PCF8563_read_regs(RTC_SECONDS_ADDR, RTC_TIMEDATE_LENGTH, data);
  if ( !result ) return false;

  m_pre_time.seconds  = data[0] & 0x7F;
  m_pre_time.minutes  = data[1] & 0x7F;
  m_pre_time.hours    = data[2] & 0x3F;
  m_pre_time.days     = data[3] & 0x3F;
  m_pre_time.weekdays = data[4] & 0x07;
  m_pre_time.months   = data[5] & 0x1F;
  m_pre_time.years    = data[6];
  return true;
}

uint8_t pcf8563_init(uint8_t scl_pin, uint8_t sda_pin)
{
  ret_code_t err_code;

  const nrf_drv_twi_config_t twi_lm75b_config = {
    .scl = scl_pin,
    .sda = sda_pin,
    .frequency = NRF_TWI_FREQ_100K,
    .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
    .clear_bus_init = false
  };

  err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
  if (err_code != NRF_SUCCESS)return false;

  nrf_drv_twi_enable(&m_twi);

  uint8_t data[2];
  data[0] = 0x00;
  data[1] = 0x00;
  PCF8563_write_regs(2, data);

  pcf8563_read();

  return true;

}