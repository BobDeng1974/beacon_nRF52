#ifndef TIMESLOT_H__
#define TIMESLOT_H__

#include "tb_global.h"


typedef struct
{
    uint32_t                adv_interval;
    bool                    keep_running;                       /** */
    bool                    is_running;                         /** is the 'beacon' running*/
    nrf_radio_request_t     timeslot_request;                   /** */
    ble_gap_addr_t          addr;                                /** ble address to be used by the beacon*/
    ble_srv_error_handler_t error_handler;                      /**< Function to be called in case of an error. */
} advertising_timeslot_param_t;


uint16_t get_timesdlot_distance(void);

/**@brief Radio GAP advertising configure
*/
uint8_t * radio_gap_adv_set_configure(ble_gap_adv_data_t const *p_adv_data, ble_gap_adv_params_t const *p_adv_params);

/**@brief Radio event handler
*/
void RADIO_timeslot_IRQHandler(void);


/**@brief Timeslot signal handler
 */
void nrf_evt_signal_handler(uint32_t evt_id);


/**@brief Timeslot event handler
 */
static nrf_radio_signal_callback_return_param_t * radio_callback(uint8_t signal_type);


/**@brief Function for initializing the timeslot API.
 */
uint32_t timeslot_start(void);


#endif