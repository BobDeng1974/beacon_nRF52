#include "timeslot.h"

#define ADV_CHANNEL_37                  37
#define ADV_CHANNEL_38                  38
#define ADV_CHANNEL_39                  39

#define FREQ_ADV_CHANNEL_37             2
#define FREQ_ADV_CHANNEL_38             26
#define FREQ_ADV_CHANNEL_39             80

#define APP_PDU_INFO_LENGTH             40    
#define ADV_PACK_LENGTH_IDX 1
#define ADV_DATA_LENGTH_IDX 12
#define ADV_HEADER_LEN      3
#define ADV_TYPE_LEN 2

#define APP_DEVICE_TYPE               0x02                              /**< 0x02 refers to Beacon. */

enum mode_t
{
  ADV_INIT,                           /** Initialisation*/
  ADV_RX_CH37,                        /** Advertising on Rx channel 37*/
  ADV_RX_CH38,                        /** Advertising on Rx channel 38*/
  ADV_RX_CH39,                        /** Advertising on Rx channel 39*/
  ADV_DONE                            /** Done advertising*/
};


static uint16_t m_tbl_adv_distance_msec_info[16] = {
  100,    200,    400,    800,    1600,   3200,   6400,   10000,
  125,    150,    175,    225,     250,    275,    300,      76
};

static uint16_t m_tbl_slot_length_msec_info[16] = {
   10,     20,     40,     80,     160,    320,    640,    1000,
   25,     50,     75,    125,     150,    175,    200,      30
};

/**Constants for timeslot API*/
static enum mode_t mode;
static uint8_t m_keep_running = false;
static uint8_t m_is_running   = false;
static nrf_radio_request_t  m_timeslot_request;
static uint32_t m_slot_length;  
static nrf_radio_signal_callback_return_param_t signal_callback_return_param;
static advertising_timeslot_param_t m_eddystone;
static uint8_t m_adv_pdu[APP_PDU_INFO_LENGTH];
static advertising_timeslot_param_t m_adv_timeslot[MAX_ADV_MODE_LIST];

#ifndef RADIO_TIMESLOT_PDU
uint8_t * radio_gap_adv_set_configure(ble_gap_adv_data_t const *p_adv_data, ble_gap_adv_params_t const *p_adv_params)
{
  int mode = get_current_advmode();

  memset(m_adv_pdu, 0x00, APP_PDU_INFO_LENGTH);

  // Packer header
  m_adv_pdu[0] = p_adv_params->properties.type;    // Advertisement type ADV_NONCONN_IND
  m_adv_pdu[1] = p_adv_data->adv_data.len + sizeof(m_device_addr.addr); 

  // Advertising Address
  m_adv_pdu[2] = m_device_addr.addr[5];
  m_adv_pdu[3] = m_device_addr.addr[4];
  m_adv_pdu[4] = m_device_addr.addr[3];
  m_adv_pdu[5] = m_device_addr.addr[2];
  m_adv_pdu[6] = m_device_addr.addr[1];
  m_adv_pdu[7] = m_device_addr.addr[0];

  // Advertising Data
  memcpy(&m_adv_pdu[8], p_adv_data->adv_data.p_data, p_adv_data->adv_data.len);
    
  return &m_adv_pdu[0];
}
#endif

/* config */
/**@brief Request next timeslot event in earliest configuration
 */
uint32_t request_next_event_earliest(void)
{
  int mode_index = get_current_advmode();
  mode_index = 0;
  advertising_timeslot_param_t *_adv_timeslot_param = &m_adv_timeslot[mode_index];
  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  uint16_t slot_length = m_tbl_slot_length_msec_info[_beacon_info[BINFO_TIMESLOT_ADV_DISTANCE_LIST_IDX+mode_index]];

  m_slot_length                                                     = slot_length * 1000;
  m_slot_length                                                     = 15000;
  _adv_timeslot_param->timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
  _adv_timeslot_param->timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
  _adv_timeslot_param->timeslot_request.params.earliest.priority    = NRF_RADIO_PRIORITY_HIGH;
  _adv_timeslot_param->timeslot_request.params.earliest.length_us   = m_slot_length;
  _adv_timeslot_param->timeslot_request.params.earliest.timeout_us  = 1000000;
  memcpy(&m_timeslot_request, &_adv_timeslot_param->timeslot_request, sizeof(nrf_radio_request_t));
  return sd_radio_request(&m_timeslot_request);
}

/**@brief Configure next timeslot event in earliest configuration
 */
void configure_next_event_earliest(void)
{
  int mode_index = get_current_advmode();
  mode_index = 0;
  advertising_timeslot_param_t *_adv_timeslot_param = &m_adv_timeslot[mode_index];
  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  uint16_t slot_length = m_tbl_slot_length_msec_info[_beacon_info[BINFO_TIMESLOT_ADV_DISTANCE_LIST_IDX+mode_index]];

  m_slot_length                                                     = slot_length * 1000;
  m_slot_length                                                     = 15000;
  _adv_timeslot_param->timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
  _adv_timeslot_param->timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
  _adv_timeslot_param->timeslot_request.params.earliest.priority    = NRF_RADIO_PRIORITY_HIGH;
  _adv_timeslot_param->timeslot_request.params.earliest.length_us   = m_slot_length;
  _adv_timeslot_param->timeslot_request.params.earliest.timeout_us  = 1000000;
  memcpy(&m_timeslot_request, &_adv_timeslot_param->timeslot_request, sizeof(nrf_radio_request_t));
}

/**@brief Configure next timeslot event in normal configuration
 */
void configure_next_event_normal(void)
{
  int mode_index = get_current_advmode();
  mode_index = 0;
  advertising_timeslot_param_t *_adv_timeslot_param = &m_adv_timeslot[mode_index];
  uint8_t *_beacon_info = ble_bms_get_beacon_info();
  uint16_t slot_length = m_tbl_slot_length_msec_info[_beacon_info[BINFO_TIMESLOT_ADV_DISTANCE_LIST_IDX+mode_index]];
  uint16_t distance_us = m_tbl_adv_distance_msec_info[_beacon_info[BINFO_TIMESLOT_ADV_DISTANCE_LIST_IDX+mode_index]];

  m_slot_length                                                    = slot_length * 1000;
  m_slot_length                                                    = 15000;
  _adv_timeslot_param->timeslot_request.request_type               = NRF_RADIO_REQ_TYPE_NORMAL;
  _adv_timeslot_param->timeslot_request.params.normal.hfclk        = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
  _adv_timeslot_param->timeslot_request.params.normal.priority     = NRF_RADIO_PRIORITY_HIGH;
  _adv_timeslot_param->timeslot_request.params.normal.distance_us  = distance_us * 1000; //to mssec
  _adv_timeslot_param->timeslot_request.params.normal.distance_us  = 100000;
  _adv_timeslot_param->timeslot_request.params.normal.length_us    = m_slot_length; //to mssec
  memcpy(&m_timeslot_request, &_adv_timeslot_param->timeslot_request, sizeof(nrf_radio_request_t));
}

/* common config */
static void radio_configure_radio()
{
  // Radio config
  NRF_RADIO->TXPOWER   = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);
  NRF_RADIO->FREQUENCY = 7UL;  // Frequency bin 7, 2407MHz
  NRF_RADIO->MODE      = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);

  NRF_RADIO->POWER        = 1;
  NRF_RADIO->PCNF0        =   (((1UL) << RADIO_PCNF0_S0LEN_Pos                               ) & RADIO_PCNF0_S0LEN_Msk)
                            | (((2UL) << RADIO_PCNF0_S1LEN_Pos                               ) & RADIO_PCNF0_S1LEN_Msk)
                            | (((6UL) << RADIO_PCNF0_LFLEN_Pos                               ) & RADIO_PCNF0_LFLEN_Msk);
  NRF_RADIO->PCNF1        =   (((RADIO_PCNF1_ENDIAN_Little)     << RADIO_PCNF1_ENDIAN_Pos    ) & RADIO_PCNF1_ENDIAN_Msk)
                            | (((3UL)                           << RADIO_PCNF1_BALEN_Pos     ) & RADIO_PCNF1_BALEN_Msk)
                            | (((0UL)                           << RADIO_PCNF1_STATLEN_Pos   ) & RADIO_PCNF1_STATLEN_Msk)
                            | ((((uint32_t) 37)                 << RADIO_PCNF1_MAXLEN_Pos    ) & RADIO_PCNF1_MAXLEN_Msk)
                            | ((RADIO_PCNF1_WHITEEN_Enabled     << RADIO_PCNF1_WHITEEN_Pos   ) & RADIO_PCNF1_WHITEEN_Msk);
  NRF_RADIO->CRCCNF       =   (((RADIO_CRCCNF_SKIPADDR_Skip)    << RADIO_CRCCNF_SKIPADDR_Pos ) & RADIO_CRCCNF_SKIPADDR_Msk)
                            | (((RADIO_CRCCNF_LEN_Three)        << RADIO_CRCCNF_LEN_Pos      ) & RADIO_CRCCNF_LEN_Msk);
  NRF_RADIO->CRCPOLY      = 0x0000065b;
  NRF_RADIO->RXADDRESSES  = ((RADIO_RXADDRESSES_ADDR0_Enabled)  << RADIO_RXADDRESSES_ADDR0_Pos);
  NRF_RADIO->SHORTS       = ((1 << RADIO_SHORTS_READY_START_Pos) | (1 << RADIO_SHORTS_END_DISABLE_Pos));
  NRF_RADIO->MODE         = ((RADIO_MODE_MODE_Ble_1Mbit)        << RADIO_MODE_MODE_Pos       ) & RADIO_MODE_MODE_Msk;
  NRF_RADIO->TIFS         = 100;
  NRF_RADIO->INTENSET     = (1 << RADIO_INTENSET_DISABLED_Pos);

  NRF_RADIO->PREFIX0      = 0x0000008e; //access_addr[3]
  NRF_RADIO->BASE0        = 0x89bed607; //access_addr[0:3]
  NRF_RADIO->BASE0        = 0x89bed600; //access_addr[0:3]
  NRF_RADIO->CRCINIT      = 0x00555555;
  NRF_RADIO->PACKETPTR    = (uint32_t) m_adv_pdu[0];

  NVIC_EnableIRQ(RADIO_IRQn);
}

void start_timer(void)
{		
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;  // Set the timer in Counter Mode
   // NRF_TIMER0->TASKS_CLEAR = 1;               // clear the task first to be usable for later
    //NRF_TIMER0->PRESCALER = 6;								 // Prescaler = 6 creates 250kHz tick frequency, i.e. tick frequency of 4us
    //NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_16Bit;//0x01UL;										//Set counter to 16 bit resolution
    NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
    NRF_TIMER0->CC[0] = m_slot_length - 5000;
    // Set and enable interrupt on Timer0
    NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
    NVIC_EnableIRQ(TIMER0_IRQn);
		
    //NRF_TIMER0->TASKS_START = 1;               // Start timer
}

void radio_handle_start(void)
{
    // Configure TX_EN on TIMER EVENT_0
  NRF_PPI->CH[8].TEP      = (uint32_t)(&NRF_RADIO->TASKS_TXEN);
  NRF_PPI->CH[8].EEP      = (uint32_t)(&NRF_TIMER0->EVENTS_COMPARE[0]);
  NRF_PPI->CHENSET        = (1 << 8);

  // Configure and initiate radio
  radio_configure_radio();
  NRF_RADIO->TASKS_DISABLE = 1;
}

static void radio_set_adv_ch(uint32_t channel)
{
  switch(channel)
  {
  case ADV_CHANNEL_37 : 
    NRF_RADIO->FREQUENCY    = FREQ_ADV_CHANNEL_37;
    NRF_RADIO->DATAWHITEIV  = ADV_CHANNEL_37;
    break;
  case ADV_CHANNEL_38 :
    NRF_RADIO->FREQUENCY    = FREQ_ADV_CHANNEL_38;
    NRF_RADIO->DATAWHITEIV  = ADV_CHANNEL_38;
    break;
 case ADV_CHANNEL_39 :
    NRF_RADIO->FREQUENCY    = FREQ_ADV_CHANNEL_39;
    NRF_RADIO->DATAWHITEIV  = ADV_CHANNEL_39;
    break;
  }
}

void handle_radio_disable(enum mode_t mode)
{
  switch (mode)
  {
  case ADV_RX_CH37:
    radio_set_adv_ch(ADV_CHANNEL_37);
    NRF_RADIO->TASKS_TXEN = 1;
    break;
  case ADV_RX_CH38:
    radio_set_adv_ch(ADV_CHANNEL_38);
    NRF_TIMER0->TASKS_CLEAR = 1;
    NRF_TIMER0->CC[0]       = 400;
    break;
  case ADV_RX_CH39:
    radio_set_adv_ch(ADV_CHANNEL_39);
    NRF_TIMER0->TASKS_CLEAR = 1;
    NRF_TIMER0->CC[0]       = 400;
    break;
  default:
    break;
  }
}


/**@brief Timeslot signal handler
 */
void nrf_evt_signal_handler(uint32_t evt_id)
{
  uint32_t err_code;

  switch (evt_id)
  {
  case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
    NRF_LOG_INFO("NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN")
    break;
  case NRF_EVT_RADIO_SESSION_IDLE:
    NRF_LOG_INFO("NRF_EVT_RADIO_SESSION_IDLE")
   if (m_is_running) {
      m_is_running = false;
      err_code = sd_radio_session_close();
      APP_ERROR_CHECK(err_code);
    }
    break;
  case NRF_EVT_RADIO_SESSION_CLOSED:
    NRF_LOG_INFO("NRF_EVT_RADIO_SESSION_CLOSED")
    break;
  case NRF_EVT_RADIO_BLOCKED:
    //NRF_LOG_INFO("NRF_EVT_RADIO_BLOCKED")
    //Fall through
  case NRF_EVT_RADIO_CANCELED:
    //NRF_LOG_INFO("NRF_EVT_RADIO_CANCELED")
    if (m_keep_running) {
      err_code = request_next_event_earliest();
      APP_ERROR_CHECK(err_code);
    }
    break;
  default:
    break;
  }
}


/**@brief Timeslot event handler
 */
static nrf_radio_signal_callback_return_param_t * radio_callback(uint8_t signal_type)
{
  //nrf_gpio_pin_toggle(DEBUG_PIN);

  signal_callback_return_param.params.request.p_next = NULL;
  signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;

  switch(signal_type)
  {
  case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
   //Start of the timeslot - set up timer interrupt
   nrf_gpio_pin_set(DEBUG_PIN);
  #ifdef TIMESLOT_RADIO_MODE
    radio_handle_start();
    mode = ADV_INIT;
    mode++;
  #else
    start_timer();
  #endif
    break;

  case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
    //Timer interrupt - do graceful shutdown - schedule next timeslot
    nrf_gpio_pin_clear(DEBUG_PIN);
    NRF_RADIO->PACKETPTR = (uint32_t) m_adv_pdu[0];

    if (m_keep_running) {
      configure_next_event_normal();
      signal_callback_return_param.params.request.p_next = &m_timeslot_request;
      signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
    }
    else {
      signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
    }
    break;

  case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
    NRF_LOG_INFO("NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO");
  #ifdef TIMESLOT_RADIO_MODE
    NRF_RADIO->PACKETPTR = (uint32_t) m_adv_pdu[0];
    if (NRF_RADIO->EVENTS_DISABLED == 1) {
      NRF_RADIO->EVENTS_DISABLED = 0;

      handle_radio_disable(mode);
      mode++;

      if (mode == ADV_DONE) {
        NRF_PPI->CHENCLR = (1 << 8);  

        if (m_keep_running){
          configure_next_event_normal();
          signal_callback_return_param.params.request.p_next = &m_timeslot_request;
          signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
        }
        else {
          NRF_LOG_INFO("TIMER0_END");
          signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
        }
      }
    }
  #endif
    break;

  case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
    //Try scheduling a new timeslot
    NRF_LOG_INFO("NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED");
    configure_next_event_earliest();
    signal_callback_return_param.params.request.p_next = &m_timeslot_request;
    signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
    break;

  case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
    NRF_LOG_INFO("NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED");
    NRF_TIMER0->EVENTS_COMPARE[0]=0;
    NRF_TIMER0->TASKS_CLEAR = 1;
    signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
   break;

  default:
    //No implementation needed
    break;
  }
  return (&signal_callback_return_param);
}


/**@brief Function for initializing the timeslot API.
 */
uint32_t timeslot_start(void)
{
  uint32_t err_code;
    
  m_keep_running = true;
  m_is_running   = true;

  err_code = sd_radio_session_open(radio_callback);
  if (err_code != NRF_SUCCESS)
  {
    return err_code;
  }
    
  err_code = request_next_event_earliest();
  if (err_code != NRF_SUCCESS)
  {
    (void)sd_radio_session_close();
  }
  return NRF_SUCCESS;
}

uint32_t timeslot_stop(void)
{
    m_keep_running = false;
}
