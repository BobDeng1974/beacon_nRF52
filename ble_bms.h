#ifndef BLE_BMS_H__
#define BLE_BMS_H__

#include "ble.h"
#include "ble_srv_common.h"
#include <stdint.h>
#include <stdbool.h>

// C2E68F69-2B17-4DAB-A139-F69D160D0001
#define BMS_UUID_BASE {0xC2, 0xE6, 0x8F, 0x69, 0x2B, 0x17, 0x4D, 0xAB, 0xA1, 0x39, 0xF6, 0x9D, 0x16, 0x0D, 0x00, 0x01}

#define BLE_UUID_BMS_SERVICE_                    0x0001
#define BLE_UUID_BMS_SETTINGS_CHARACTERISTIC     0x0002
#define BLE_UUID_BMS_DFU_CHARACTERISTIC          0x0010
#define BLE_UUID_BMS_LED_CHARACTERISTIC          0x0011
#define BLE_UUID_BMS_FCM_CHARACTERISTIC          0x0012
#define BLE_UUID_BMS_BCM_CHARACTERISTIC          0x0013

#define BLE_BMS_MAX_DATA_LEN            (BLE_GATT_ATT_MTU_DEFAULT - 3)  /**< Maximum length of data (in bytes) that can be transmitted 
                                                                        by the Nordic UART service module to the peer. */
#define BLE_BMS_MAX_SETTINGS_CHAR_LEN         BLE_BMS_MAX_DATA_LEN   /**< Maximum length of the UUID Characteristic (in bytes). */

// Forward declaration of the ble_bms_t type.
typedef struct ble_bms_s ble_bms_t;

/**@brief Beacon Management Service event handler type. */
typedef void (*ble_bms_data_handler_t) (ble_bms_t * p_bms, uint8_t * data, uint16_t length, uint16_t handle);

/**@brief   Beacon Management Service init structure.
 *
 * @details This structure contains the initialization information for the service. The application
 *          needs to fill this structure and pass it to the service using the @ref ble_bms_init
 *          function.
 */
typedef struct
{
    ble_bms_data_handler_t   data_handler;            /**< Event handler to be called for handling received data. */
} ble_bms_init_t;

/**@brief   Beacon Management Service structure.
 *
 * @details This structure contains status information related to the service.
 */
typedef struct ble_bms_s
{
    uint8_t                  uuid_type;               /**< UUID type for Beacon Management Service Base UUID. */
    uint16_t                 service_handle;          /**< Handle of Beacon Management Service (as provided by the S110 SoftDevice). */
    ble_gatts_char_handles_t settings_handles;
    ble_gatts_char_handles_t dfu_handles;
    ble_gatts_char_handles_t led_handles;
    ble_gatts_char_handles_t fcm_handles;
    ble_gatts_char_handles_t bcm_handles;
    uint16_t                 conn_handle;             /**< Handle of the current connection (as provided by the S110 SoftDevice). 
                                                         This will be BLE_CONN_HANDLE_INVALID if not in a connection. */
    bool                     is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the all characteristic.*/
    ble_bms_data_handler_t   data_handler;            /**< Event handler to be called for handling received data. */
} ble_bms_t;

/**@brief       Function for initializing the Beacon Management Service.
 *
 * @param[out]  p_bms       Beacon Management Service structure. This structure will have to be supplied
 *                          by the application. It will be initialized by this function and will
 *                          later be used to identify this particular service instance.
 * @param[in]   p_bms_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 *              This function returns NRF_ERROR_NULL if either of the pointers p_bms or p_bms_init
 *              is NULL.
 */
//uint32_t ble_bms_init(ble_bms_t * p_nus, const ble_bms_init_t * p_bms_init);
uint32_t ble_bms_init(ble_bms_t *p_bms);

/**@brief       Beacon Management Service BLE event handler.
 *
 * @details     The Beacon Management service expects the application to call this function each time an
 *              event is received from the S110 SoftDevice. This function processes the event if it
 *              is relevant for it and calls the Beacon Management Service event handler of the
 *              application if necessary.
 *
 * @param[in]   p_bms      Beacon Management Service structure.
 * @param[in]   p_ble_evt  Event received from the S110 SoftDevice.
 */
void ble_bms_on_ble_evt(ble_bms_t * p_bms, ble_evt_t * p_ble_evt);


///**@brief       Function for sending a string to the peer.
// *
// * @details     This function will send the input string as a RX characteristic notification to the
// *              peer.
// *
// * @param[in]   p_bms          Pointer to the Nordic UART Service structure.
// * @param[in]   string         String to be sent.
// * @param[in]   length         Length of string.
// *
// * @return      NRF_SUCCESS if the DFU Service has successfully requested the S110 SoftDevice to
// *              send the notification. Otherwise an error code.
// *              This function returns NRF_ERROR_INVALID_STATE if the device is not connected to a
// *              peer or if the notification of the RX characteristic was not enabled by the peer.
// *              It returns NRF_ERROR_NULL if the pointer p_nus is NULL.
// */
//uint32_t ble_bms_send_string(ble_bms_t * p_bms, uint8_t * string, uint16_t length);

uint8_t get_status_flags(void);

void set_timeslot_mode(void);

#endif
