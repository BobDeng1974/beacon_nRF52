#define _BEACON_DEVICE_MAIN_CODE_

#include "beacon_device.h"
#include "nrf_drv_saadc.h"


/* LED */
static uint8_t m_Blink_LED_count;
static uint8_t m_Blink_Error_LED_count;
static uint8_t m_Execute_led_flash_type1;

/* Battery */
// Input range of internal Vdd measurement = (0.6 V)/(1/6) = 3.6 V
// 3.0 volts -> 14486 ADC counts with 14-bit sampling: 4828.8 counts per volt
#define ADC12_COUNTS_PER_VOLT 4551

uint16_t m_Battery_Voltage_Max_Capacity;

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
    nrf_gpio_pin_clear(LED_R);
  }
}
  #else
void execute_led(const char *param)
{
  switch(m_hardware_type) {
  case HW_TYPE_MINEW_USB_BEACON :   // MINEW USB Beacon
    return;
  case HW_TYPE_TANGERINE_BEACON :   // Tangerine Beacon
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
    break;
  case HW_TYPE_MINEW_MAX_BEACON :   // MINEW MAX Beacon
    if (param[0] == '1') {
      nrf_gpio_pin_set(LED_G_HW_MIMAXK);
      nrf_gpio_pin_clear(LED_R_HW_MIMAXK);
      nrf_gpio_pin_clear(LED_B_HW_MIMAXK);
    }
    else {
      nrf_gpio_pin_clear(LED_G_HW_MIMAXK);
      nrf_gpio_pin_clear(LED_R_HW_MIMAXK);
      nrf_gpio_pin_clear(LED_B_HW_MIMAXK);
    }
    break;
  }
}

void execute_error_led(const char *param)
{
  switch(m_hardware_type) {
  case HW_TYPE_MINEW_USB_BEACON :   // MINEW USB Beacon
    return;
  case HW_TYPE_TANGERINE_BEACON :   // Tangerine Beacon
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
    break;
  case HW_TYPE_MINEW_MAX_BEACON :   // MINEW MAX Beacon
    if (param[0] == '1') {
      nrf_gpio_pin_clear(LED_G_HW_MIMAXK);
      nrf_gpio_pin_set(LED_R_HW_MIMAXK);
      nrf_gpio_pin_clear(LED_B_HW_MIMAXK);
    }
    else {
      nrf_gpio_pin_clear(LED_G_HW_MIMAXK);
      nrf_gpio_pin_clear(LED_R_HW_MIMAXK);
      nrf_gpio_pin_clear(LED_B_HW_MIMAXK);
    }
    break;
  }
}

void execute_pending_led(const char *param)
{
  switch(m_hardware_type) {
  case HW_TYPE_MINEW_USB_BEACON :   // MINEW USB Beacon
    return;
  case HW_TYPE_TANGERINE_BEACON :   // Tangerine Beacon
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
    break;
  case HW_TYPE_MINEW_MAX_BEACON :   // MINEW MAX Beacon
    if (param[0] == '1') {
      nrf_gpio_pin_clear(LED_G_HW_MIMAXK);
      nrf_gpio_pin_clear(LED_R_HW_MIMAXK);
      nrf_gpio_pin_set(LED_B_HW_MIMAXK);
    }
    else {
      nrf_gpio_pin_clear(LED_G_HW_MIMAXK);
      nrf_gpio_pin_clear(LED_R_HW_MIMAXK);
      nrf_gpio_pin_clear(LED_B_HW_MIMAXK);
    }
    break;
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
    nrf_delay_ms(100);

    execute_error_led(LED_OFF);
    nrf_delay_ms(100);
  }
}

void blink_pending_led(uint8_t c, uint8_t t)
{
  for(uint8_t i = 0; i < c; i++) 
  {
    execute_pending_led(LED_ON);
    nrf_delay_ms(t);

    execute_pending_led(LED_OFF);
    nrf_delay_ms(t);
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
#ifdef DEBUG_PIN1_ENABLE
  nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0,9));
  nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(0,9));
  nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0,10));
  nrf_gpio_pin_set(NRF_GPIO_PIN_MAP(0,10));
#endif

  nrf_gpio_cfg_output(LED_G);
  nrf_gpio_cfg_output(LED_R);
  nrf_gpio_cfg_output(LED_B);
  nrf_gpio_pin_clear(LED_G);
  nrf_gpio_pin_set(LED_R);
  nrf_gpio_pin_set(LED_B);
}

void gpiote_init_hw_type(uint8_t hw_type)
{
  switch(hw_type) {
  case HW_TYPE_MINEW_USB_BEACON :   // MINEW USB Beacon
    nrf_gpio_cfg_input(LED_R, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(LED_G, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_input(LED_B, NRF_GPIO_PIN_NOPULL);
    break;
  case HW_TYPE_TANGERINE_BEACON :   // Tangerine Beacon
    break;
  case HW_TYPE_MINEW_MAX_BEACON :   // MINEW MAX Beacon
    nrf_gpio_cfg_output(LED_R_HW_MIMAXK);
    nrf_gpio_cfg_output(LED_G_HW_MIMAXK);
    nrf_gpio_cfg_output(LED_B_HW_MIMAXK);
    nrf_gpio_cfg_input(SW_HW_MIMAXK, NRF_GPIO_PIN_PULLUP);
    break;
  case HW_TYPE_NORDIC_NRF52DK :     // Nordic nRF52-DK Beacon
    nrf_gpio_cfg_output(LED_R_HW_52DK);
    nrf_gpio_cfg_output(LED_G_HW_52DK);
    nrf_gpio_cfg_output(LED_B_HW_52DK);
    break;
  }
}

/**
 * @brief Function for 14-bit adc init in polled mode
 */
void battery_init(void)
{
  if ( m_hardware_type == HW_TYPE_TANGERINE_BEACON ) 
  {
    nrf_saadc_channel_config_t adc_channel_conf = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(BAT_V);
    ret_code_t err_code = nrf_drv_saadc_channel_init(0, &adc_channel_conf);
    APP_ERROR_CHECK(err_code);
  }
  else {
    nrf_saadc_channel_config_t myConfig =
    {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain       = NRF_SAADC_GAIN1_6,
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,
        .acq_time   = NRF_SAADC_ACQTIME_40US,
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .burst      = NRF_SAADC_BURST_ENABLED,
        .pin_p      = NRF_SAADC_INPUT_VDD,
        .pin_n      = NRF_SAADC_INPUT_DISABLED
    };

    nrf_saadc_resolution_set((nrf_saadc_resolution_t) 3);   // 3 is 14-bit
    nrf_saadc_oversample_set((nrf_saadc_oversample_t) 2);   // 2 is 4x, about 150uSecs total
    nrf_saadc_int_disable(NRF_SAADC_INT_ALL);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
    nrf_saadc_enable();

    NRF_SAADC->CH[1].CONFIG =
              ((myConfig.resistor_p << SAADC_CH_CONFIG_RESP_Pos)   & SAADC_CH_CONFIG_RESP_Msk)
            | ((myConfig.resistor_n << SAADC_CH_CONFIG_RESN_Pos)   & SAADC_CH_CONFIG_RESN_Msk)
            | ((myConfig.gain       << SAADC_CH_CONFIG_GAIN_Pos)   & SAADC_CH_CONFIG_GAIN_Msk)
            | ((myConfig.reference  << SAADC_CH_CONFIG_REFSEL_Pos) & SAADC_CH_CONFIG_REFSEL_Msk)
            | ((myConfig.acq_time   << SAADC_CH_CONFIG_TACQ_Pos)   & SAADC_CH_CONFIG_TACQ_Msk)
            | ((myConfig.mode       << SAADC_CH_CONFIG_MODE_Pos)   & SAADC_CH_CONFIG_MODE_Msk)
            | ((myConfig.burst      << SAADC_CH_CONFIG_BURST_Pos)  & SAADC_CH_CONFIG_BURST_Msk);

    NRF_SAADC->CH[1].PSELN = myConfig.pin_n;
    NRF_SAADC->CH[1].PSELP = myConfig.pin_p;
  }
}

/**
 * @brief Function for 14-bit adc battery voltage by direct blocking reading
 */
uint16_t get_battery_level()
{
  if ( m_hardware_type == HW_TYPE_TANGERINE_BEACON ) 
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
  else {
    uint16_t result = 0xFFFF;           // Some recognisable dummy value
    uint32_t timeout = 10000;           // Trial and error
    volatile int16_t buffer[8];
  
    // Enable command
    nrf_saadc_enable();
    NRF_SAADC->RESULT.PTR = (uint32_t)buffer;
    NRF_SAADC->RESULT.MAXCNT = 1;
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
    nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);

    while (0 == nrf_saadc_event_check(NRF_SAADC_EVENT_END) && timeout > 0)
    {
      //nrf_delay_us(10);
      timeout--;
    }
    nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
  
    // Disable command to reduce power consumption
    nrf_saadc_disable();
    if (timeout != 0) {
      result = ((buffer[0] * 1000L)+(ADC12_COUNTS_PER_VOLT/2)) / ADC12_COUNTS_PER_VOLT;
    }
    //NRF_LOG_INFO("ADC12 = %dmV(%d) Timeout(%d)", result, buffer[0], timeout);
    return result;
 }
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

uint8_t battery_level_to_percent(const uint16_t mvolts)
{
  float fMaxCapacity = (float)m_Battery_Voltage_Max_Capacity;

  fMaxCapacity = ((float)mvolts / fMaxCapacity) * 100.0;
  uint8_t battery_level = (uint8_t)fMaxCapacity;
  //NRF_LOG_INFO("Battery = %d", battery_level);
  if ( battery_level > 100 ) battery_level = 100;
  return battery_level;
}

/*
  0x0B: 100%: 2650 <=
  0x0A:  90%
  0x09:  80%
  0x08:  70%
  0x07:  60%
  0x06:  50%
  0x05:  40%
  0x04:  30%
  0x03:  20%
  0x02:  10%
  0x01:   0%
  0x00: Unkown/continuous power
*/
uint8_t battery_level_to_percent_devidedby10(const uint16_t mvolts)
{
  uint8_t battery_level;

  if (mvolts >= 99) {           // 100%
    battery_level = 0x0B;
  } else if (mvolts >= 90) {    // 90%
    battery_level = 0x0A;
  } else if (mvolts >= 80) {    // 80%
    battery_level = 0x09;
  } else if (mvolts >= 70) {    // 70%
    battery_level = 0x08;
  } else if (mvolts >= 60) {    // 60%
    battery_level = 0x07;
  } else if (mvolts >= 50) {    // 50%
    battery_level = 0x06;
  } else if (mvolts >= 40) {    // 40%
    battery_level = 0x05;
  } else if (mvolts >= 30) {    // 30%
    battery_level = 0x04;
  } else if (mvolts >= 20) {    // 20%
    battery_level = 0x03;
  } else if (mvolts >= 10) {    // 10%
    battery_level = 0x02;
  } else {                        // 0%
    battery_level = 0x01;
  }

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
  if (err_code != NRF_SUCCESS)return false;
  while (m_xfer_done == false) {
    if (m_xfer_done == 0xFF) return false;
  }

  m_xfer_done = false;
  err_code = nrf_drv_twi_rx(&m_twi, RTC_SLAVE_ADDR, values, length);
  if (err_code != NRF_SUCCESS)return false;
  while (m_xfer_done == false) {
    if (m_xfer_done == 0xFF) return false;
  }
  return true;
}

uint8_t PCF8563_write_regs(uint8_t length, uint8_t *values)
{
  m_xfer_done = false;
  ret_code_t err_code = nrf_drv_twi_tx(&m_twi, RTC_SLAVE_ADDR, values, length, false);
  if (err_code != NRF_SUCCESS)return false;
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
  if (m_hardware_type != HW_TYPE_TANGERINE_BEACON) return false;   

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