#include "timeslot.h"

#define ADV_PACK_LENGTH_IDX             1
#define ADV_DATA_LENGTH_IDX             12
#define ADV_HEADER_LEN                  3
#define ADV_TYPE_LEN                    2
#define APP_DEVICE_TYPE                 0x02                              /**< 0x02 refers to Beacon. */
#define APP_PDU_INFO_LENGTH             40
#define BEACON_SLOT_LENGTH              5500

#define ADV_CHANNEL_37                  37
#define ADV_CHANNEL_38                  38
#define ADV_CHANNEL_39                  39

#define FREQ_ADV_CHANNEL_37             2
#define FREQ_ADV_CHANNEL_38             26
#define FREQ_ADV_CHANNEL_39             80

static struct
{
    uint32_t                adv_interval;                   /** Advertising interval in milliseconds to be used for 'beacon' advertisements*/
    bool                    keep_running;                   /** */
    bool                    is_running;                     /** is the 'beacon' running*/
    uint32_t                slot_length;                    /** */
    nrf_radio_request_t     timeslot_request;               /** */
    ble_srv_error_handler_t error_handler;                  /**< Function to be called in case of an error. */
} m_beacon;

enum mode_t
{
    ADV_INIT,                                               /** Initialisation*/
    ADV_RX_CH37,                                            /** Advertising on Rx channel 37*/
    ADV_RX_CH38,                                            /** Advertising on Rx channel 38*/
    ADV_RX_CH39,                                            /** Advertising on Rx channel 39*/
    ADV_DONE                                                /** Done advertising*/
};

static  uint8_t m_adv_pdu[APP_PDU_INFO_LENGTH];
static  uint8_t line_packet_sw = false;


nrf_radio_request_t * m_configure_next_event(void)
{
    m_beacon.timeslot_request.request_type              = NRF_RADIO_REQ_TYPE_NORMAL;
    m_beacon.timeslot_request.params.normal.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_beacon.timeslot_request.params.normal.priority    = NRF_RADIO_PRIORITY_NORMAL;
    m_beacon.timeslot_request.params.normal.distance_us = m_beacon.adv_interval * 10;
    m_beacon.timeslot_request.params.normal.length_us   = m_beacon.slot_length;
    return &m_beacon.timeslot_request;
}

uint32_t m_request_earliest(enum NRF_RADIO_PRIORITY priority)
{
    m_beacon.timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_beacon.timeslot_request.params.earliest.hfclk       = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_beacon.timeslot_request.params.earliest.priority    = priority;
    m_beacon.timeslot_request.params.earliest.length_us   = m_beacon.slot_length;
    m_beacon.timeslot_request.params.earliest.timeout_us  = m_beacon.adv_interval * 10;
    return sd_radio_request(&m_beacon.timeslot_request);
}

uint32_t m_request_normal(void)
{
    m_beacon.timeslot_request.request_type                = NRF_RADIO_REQ_TYPE_NORMAL;
    m_beacon.timeslot_request.params.normal.hfclk         = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_beacon.timeslot_request.params.normal.priority      = NRF_RADIO_PRIORITY_NORMAL;
    m_beacon.timeslot_request.params.normal.distance_us   = m_beacon.adv_interval * 10 * 2;
    m_beacon.timeslot_request.params.normal.length_us     = m_beacon.slot_length;
    return sd_radio_request(&m_beacon.timeslot_request);
}

void radio_pdu_configure(ble_gap_adv_data_t const *p_adv_data, ble_gap_adv_params_t const *p_adv_params,  uint8_t *adv_pdu)
{
  memset(adv_pdu, 0x00, APP_PDU_INFO_LENGTH);

  // Packer header
  *adv_pdu = 0x60;                             // Advertisement type ADV_NONCONN_IND
  *(adv_pdu+1) = p_adv_data->adv_data.len + sizeof(m_device_addr.addr); 

  // Advertising Address
  memcpy(adv_pdu+3, m_device_addr.addr, 6);
  *(adv_pdu+3) = *(adv_pdu+3) + 1;

  // Advertising Data
  memcpy(adv_pdu+9, p_adv_data->adv_data.p_data, p_adv_data->adv_data.len);
}

uint8_t * radio_gap_adv_set_configure(ble_gap_adv_data_t const *p_adv_data)
{
  memset(m_adv_pdu, 0x00, APP_PDU_INFO_LENGTH);

  // Packer header
  m_adv_pdu[0] = 0x60;                             // Advertisement type ADV_NONCONN_IND
  m_adv_pdu[1] = p_adv_data->adv_data.len + sizeof(m_device_addr.addr); 

  // Advertising Address
  m_adv_pdu[3] = m_device_addr.addr[0] + 1;
  m_adv_pdu[4] = m_device_addr.addr[1];
  m_adv_pdu[5] = m_device_addr.addr[2];
  m_adv_pdu[6] = m_device_addr.addr[3];
  m_adv_pdu[7] = m_device_addr.addr[4];
  m_adv_pdu[8] = m_device_addr.addr[5];

  // Advertising Data
  memcpy(&m_adv_pdu[9], p_adv_data->adv_data.p_data, p_adv_data->adv_data.len);
    
  return &m_adv_pdu[0];
}

static void m_set_adv_ch(uint32_t channel)
{
    if (channel == ADV_CHANNEL_37)
    {
        NRF_RADIO->FREQUENCY    = FREQ_ADV_CHANNEL_37;
        NRF_RADIO->DATAWHITEIV  = ADV_CHANNEL_37;
    }
    if (channel == ADV_CHANNEL_38)
    {
        NRF_RADIO->FREQUENCY    = FREQ_ADV_CHANNEL_38;
        NRF_RADIO->DATAWHITEIV  = ADV_CHANNEL_38;
    }
    if (channel == ADV_CHANNEL_39)
    {
        NRF_RADIO->FREQUENCY    = FREQ_ADV_CHANNEL_39;
        NRF_RADIO->DATAWHITEIV  = ADV_CHANNEL_39;
    }
}


static void m_configure_radio()
{
    uint8_t * p_adv_pdu = get_line_beacon_packet();
    if (line_packet_sw) p_adv_pdu = get_line_ibeacon_packet();
    line_packet_sw = !line_packet_sw;

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
    NRF_RADIO->TIFS         = 150;
    NRF_RADIO->INTENSET     = (1 << RADIO_INTENSET_DISABLED_Pos);
    NRF_RADIO->PREFIX0      = 0x0000008e; //access_addr[3]
    NRF_RADIO->BASE0        = 0x89bed600; //access_addr[0:3]
    NRF_RADIO->CRCINIT      = 0x00555555;
    NRF_RADIO->PACKETPTR    = (uint32_t) p_adv_pdu;
    
    NVIC_EnableIRQ(RADIO_IRQn);
}

void m_handle_start(void)
{
    // Configure TX_EN on TIMER EVENT_0
    NRF_PPI->CH[8].TEP    = (uint32_t)(&NRF_RADIO->TASKS_TXEN);
    NRF_PPI->CH[8].EEP    = (uint32_t)(&NRF_TIMER0->EVENTS_COMPARE[0]);
    NRF_PPI->CHENSET      = (1 << 8);
    
    // Configure and initiate radio
    m_configure_radio();
    NRF_RADIO->TASKS_DISABLE = 1;
}

void m_handle_radio_disabled(enum mode_t mode)
{
    switch (mode)
    {
        case ADV_RX_CH37:
            m_set_adv_ch(ADV_CHANNEL_37);
            NRF_RADIO->TASKS_TXEN = 1;
            break;
        case ADV_RX_CH38:
            m_set_adv_ch(ADV_CHANNEL_38);
            NRF_TIMER0->TASKS_CLEAR = 1;
            NRF_TIMER0->CC[0]       = 400;
            break;
        case ADV_RX_CH39:
            m_set_adv_ch(ADV_CHANNEL_39);
            NRF_TIMER0->TASKS_CLEAR = 1;
            NRF_TIMER0->CC[0]       = 400;
            break;
        default:
            break;
    }
}

static nrf_radio_signal_callback_return_param_t * m_timeslot_callback(uint8_t signal_type)
{
  static nrf_radio_signal_callback_return_param_t signal_callback_return_param;
  static enum mode_t mode;

  //nrf_gpio_pin_toggle(9);

  signal_callback_return_param.params.request.p_next  = NULL;
  signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;

  switch (signal_type)
  {
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:

      m_handle_start();

      mode = ADV_INIT;
      mode++;
      break;
    case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
      if (NRF_RADIO->EVENTS_DISABLED == 1)
      {
        NRF_RADIO->EVENTS_DISABLED = 0;

        m_handle_radio_disabled(mode);

        if (mode == ADV_DONE)
        {
            NRF_PPI->CHENCLR = (1 << 8);
            if (m_beacon.keep_running)
            {
                signal_callback_return_param.params.request.p_next = m_configure_next_event();
                signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
            }
            else
            {
                signal_callback_return_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
            }
            break;
        }
        mode++;
      }
      break;
    default:
        if (m_beacon.error_handler != NULL)
        {
            m_beacon.error_handler(NRF_ERROR_INVALID_STATE);
        }
      break;
  }

  return ( &signal_callback_return_param );
}

void timeslot_on_sys_evt(uint32_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case NRF_EVT_RADIO_SESSION_IDLE:
            if (m_beacon.is_running)
            {
                NRF_LOG_INFO("sd_radio_session_close");
                err_code = sd_radio_session_close();
                if ((err_code != NRF_SUCCESS) && (m_beacon.error_handler != NULL))
                {
                    m_beacon.error_handler(err_code);
                }
                m_beacon.is_running = false;
            }
            break;
        case NRF_EVT_RADIO_SESSION_CLOSED:
            break;
        case NRF_EVT_RADIO_BLOCKED:
        case NRF_EVT_RADIO_CANCELED: // Fall through
            if (m_beacon.keep_running)
            {
                // TODO: A proper solution should try again in <block_count> * m_beacon.adv_interval
                // err_code = m_request_earliest(NRF_RADIO_PRIORITY_HIGH);
                err_code = m_request_normal();
                if ((err_code != NRF_SUCCESS) && (m_beacon.error_handler != NULL))
                {
                    m_beacon.error_handler(err_code);
                }
            }
            break;
        case NRF_EVT_HFCLKSTARTED :                         /**< Event indicating that the HFCLK has started. */
            NRF_LOG_INFO("NRF_EVT_HFCLKSTARTED")
            break;
        case NRF_EVT_POWER_FAILURE_WARNING :                /**< Event indicating that a power failure warning has occurred. */
            NRF_LOG_INFO("NRF_EVT_POWER_FAILURE_WARNING")
            break;
        case NRF_EVT_FLASH_OPERATION_ERROR :                /**< Event indicating that the ongoing flash operation has timed out with an error. */
            NRF_LOG_INFO("NRF_EVT_FLASH_OPERATION_ERROR")
            break;
        default:
            break;
    }
}

void timeslot_init(ble_beacon_init_t * p_init)
{
    m_beacon.adv_interval = p_init->adv_interval;
    m_beacon.slot_length  = BEACON_SLOT_LENGTH;
    m_beacon.error_handler= p_init->error_handler;
}

void timeslot_start(void)
{
    m_beacon.keep_running = true;
    m_beacon.is_running   = true;

    uint32_t err_code = sd_radio_session_open(m_timeslot_callback);
    if ((err_code != NRF_SUCCESS) && (m_beacon.error_handler != NULL))
    {
        m_beacon.error_handler(err_code);
    }
    
    err_code = m_request_earliest(NRF_RADIO_PRIORITY_NORMAL);
    if ((err_code != NRF_SUCCESS) && (m_beacon.error_handler != NULL))
    {
        m_beacon.error_handler(err_code);
    }
}

void timeslot_stop(void)
{
    m_beacon.keep_running = false;
}

uint8_t get_timeslot(void)
{
    return m_beacon.is_running;
}