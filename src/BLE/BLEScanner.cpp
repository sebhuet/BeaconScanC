/*
 * BLEScanner.cpp
 *
 *  Created on: 8 sept. 2017
 *      Author: seb
 */
extern "C" {
#include "app_error.h"
#include "ble_db_discovery.h"
#include "app_util.h"
#include "boards.h"
#include "bsp_btn_ble.h"
#include "ble_hci.h"
#include "softdevice_handler.h"
#include "Memory.h"
}

#include "BLEScanner.hpp"



BLEScanner *myBleScanner;



BLEScanner::BLEScanner() {
	myBleScanner = this;
}

void BLEScanner::ble_evt_dispatch(ble_evt_t * p_ble_evt) {
	//    nrf_ble_gatt_on_ble_evt(&m_gatt, p_ble_evt);
	//    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
	//    ble_nus_c_on_ble_evt(&m_ble_nus_c,p_ble_evt);
}



//#define CENTRAL_LINK_COUNT      1                                       /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
//#define PERIPHERAL_LINK_COUNT   0                                       /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
//#define CONN_CFG_TAG            1                                       /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
//
//#define UART_TX_BUF_SIZE        256                                     /**< UART TX buffer size. */
//#define UART_RX_BUF_SIZE        256                                     /**< UART RX buffer size. */
//
//#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN              /**< UUID type for the Nordic UART Service (vendor specific). */

//#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS)         /**< Determines minimum connection interval in millisecond. */
//#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS)         /**< Determines maximum connection interval in millisecond. */
//#define SLAVE_LATENCY           0                                       /**< Determines slave latency in counts of connection events. */
//#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Determines supervision time-out in units of 10 millisecond. */
//
//#define UUID16_SIZE             2                                       /**< Size of 16 bit UUID */
//#define UUID32_SIZE             4                                       /**< Size of 32 bit UUID */
//#define UUID128_SIZE            16                                      /**< Size of 128 bit UUID */
//
//#define ECHOBACK_BLE_UART_DATA  1                                       /**< Echo the UART data that is received over the Nordic UART Service back to the sender. */

//static ble_nus_c_t              m_ble_nus_c;                            /**< Instance of NUS service. Must be passed to all NUS_C API calls. */
//static nrf_ble_gatt_t           m_gatt;                                 /**< GATT module instance. */
//static ble_db_discovery_t       m_ble_db_discovery;                     /**< Instance of database discovery module. Must be passed to all db_discovert API calls */
//static uint16_t                 m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

///**@brief Connection parameters requested for connection. */
//static ble_gap_conn_params_t const m_connection_param =
//  {
//    (uint16_t)MIN_CONNECTION_INTERVAL,  // Minimum connection
//    (uint16_t)MAX_CONNECTION_INTERVAL,  // Maximum connection
//    (uint16_t)SLAVE_LATENCY,            // Slave latency
//    (uint16_t)SUPERVISION_TIMEOUT       // Supervision time-out
//  };

///**@brief NUS uuid. */
//static ble_uuid_t const m_nus_uuid =
//{
//    /*.uuid = */BLE_UUID_NUS_SERVICE,
//    /*.type = */NUS_SERVICE_UUID_TYPE
//};

///**@brief Function for handling database discovery events.
// *
// * @details This function is callback function to handle events from the database discovery module.
// *          Depending on the UUIDs that are discovered, this function should forward the events
// *          to their respective services.
// *
// * @param[in] p_event  Pointer to the database discovery event.
// */
//static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
//{
//    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
//}

///**@brief Function for handling characters received by the Nordic UART Service.
// *
// * @details This function takes a list of characters of length data_len and prints the characters out on UART.
// *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
// */
//static void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len)
//{
//    ret_code_t ret_val;
//
//    NRF_LOG_DEBUG("Receiving data.\r\n");
//    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);
//
//    for (uint32_t i = 0; i < data_len; i++)
//    {
//        do
//        {
//            ret_val = app_uart_put(p_data[i]);
//            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
//            {
//                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.\r\n", i);
//                APP_ERROR_CHECK(ret_val);
//            }
//        } while (ret_val == NRF_ERROR_BUSY);
//    }
//    if (p_data[data_len-1] == '\r')
//    {
//        while (app_uart_put('\n') == NRF_ERROR_BUSY);
//    }
//    if (ECHOBACK_BLE_UART_DATA)
//    {
//        // Send data back to peripheral.
//        do
//        {
//            ret_val = ble_nus_c_string_send(&m_ble_nus_c, p_data, data_len);
//            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
//            {
//                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. \r\n", ret_val);
//                APP_ERROR_CHECK(ret_val);
//            }
//        } while (ret_val == NRF_ERROR_BUSY);
//    }
//}

///**@brief   Function for handling app_uart events.
// *
// * @details This function will receive a single character from the app_uart module and append it to
// *          a string. The string will be be sent over BLE when the last character received was a
// *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
// */
//void uart_event_handle(app_uart_evt_t * p_event)
//{
//    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
//    static uint16_t index = 0;
//    uint32_t ret_val;
//
//    switch (p_event->evt_type)
//    {
//        /**@snippet [Handling data from UART] */
//        case APP_UART_DATA_READY:
//            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
//            index++;
//
//            if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
//            {
//                NRF_LOG_DEBUG("Ready to send data over BLE NUS\r\n");
//                NRF_LOG_HEXDUMP_DEBUG(data_array, index);
//
//                do
//                {
//                    ret_val = ble_nus_c_string_send(&m_ble_nus_c, data_array, index);
//                    if ( (ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_BUSY) )
//                    {
//                        APP_ERROR_CHECK(ret_val);
//                    }
//                } while (ret_val == NRF_ERROR_BUSY);
//
//                index = 0;
//            }
//            break;
//
//        /**@snippet [Handling data from UART] */
//        case APP_UART_COMMUNICATION_ERROR:
//            NRF_LOG_ERROR("Communication error occurred while handling UART.\r\n");
//            APP_ERROR_HANDLER(p_event->data.error_communication);
//            break;
//
//        case APP_UART_FIFO_ERROR:
//            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.\r\n");
//            APP_ERROR_HANDLER(p_event->data.error_code);
//            break;
//
//        default:
//            break;
//    }
//}

///**@brief Callback handling NUS Client events.
// *
// * @details This function is called to notify the application of NUS client events.
// *
// * @param[in]   p_ble_nus_c   NUS Client Handle. This identifies the NUS client
// * @param[in]   p_ble_nus_evt Pointer to the NUS Client event.
// */
//
///**@snippet [Handling events from the ble_nus_c module] */
//static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, const ble_nus_c_evt_t * p_ble_nus_evt)
//{
//    ret_code_t err_code;
//
//    switch (p_ble_nus_evt->evt_type)
//    {
//        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
//            NRF_LOG_INFO("Discovery complete.\r\n");
//            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
//            APP_ERROR_CHECK(err_code);
//
//            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
//            APP_ERROR_CHECK(err_code);
//            printf("Connected to device with Nordic UART Service.\r\n");
//            break;
//
//        case BLE_NUS_C_EVT_NUS_TX_EVT:
//            ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
//            break;
//
//        case BLE_NUS_C_EVT_DISCONNECTED:
//            NRF_LOG_INFO("Disconnected.\r\n");
//            scan_start();
//            break;
//    }
//}

///**@brief Reads an advertising report and checks if a UUID is present in the service list.
// *
// * @details The function is able to search for 16-bit, 32-bit and 128-bit service UUIDs.
// *          To see the format of a advertisement packet, see
// *          https://www.bluetooth.org/Technical/AssignedNumbers/generic_access_profile.htm
// *
// * @param[in]   p_target_uuid The UUID to search for.
// * @param[in]   p_adv_report  Pointer to the advertisement report.
// *
// * @retval      true if the UUID is present in the advertisement report. Otherwise false
// */
//static bool is_uuid_present(ble_uuid_t               const * p_target_uuid,
//                            ble_gap_evt_adv_report_t const * p_adv_report)
//{
//    ret_code_t   err_code;
//    ble_uuid_t   extracted_uuid;
//    uint16_t     index  = 0;
//    uint8_t    * p_data = (uint8_t *)p_adv_report->data;
//
//    while (index < p_adv_report->dlen)
//    {
//        uint8_t field_length = p_data[index];
//        uint8_t field_type   = p_data[index + 1];
//
//        if (   (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE)
//            || (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE))
//        {
//            for (uint32_t i = 0; i < (field_length / UUID16_SIZE); i++)
//            {
//                err_code = sd_ble_uuid_decode(UUID16_SIZE,
//                                              &p_data[i * UUID16_SIZE + index + 2],
//                                              &extracted_uuid);
//
//                if (err_code == NRF_SUCCESS)
//                {
//                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
//                        && (extracted_uuid.type == p_target_uuid->type))
//                    {
//                        return true;
//                    }
//                }
//            }
//        }
//        else if (   (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_MORE_AVAILABLE)
//                 || (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_COMPLETE))
//        {
//            for (uint32_t i = 0; i < (field_length / UUID32_SIZE); i++)
//            {
//                err_code = sd_ble_uuid_decode(UUID32_SIZE,
//                                              &p_data[i * UUID32_SIZE + index + 2],
//                                              &extracted_uuid);
//
//                if (err_code == NRF_SUCCESS)
//                {
//                    if (   (extracted_uuid.uuid == p_target_uuid->uuid)
//                        && (extracted_uuid.type == p_target_uuid->type))
//                    {
//                        return true;
//                    }
//                }
//            }
//        }
//
//        else if (   (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE)
//                 || (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE))
//        {
//            err_code = sd_ble_uuid_decode(UUID128_SIZE, &p_data[index + 2], &extracted_uuid);
//            if (err_code == NRF_SUCCESS)
//            {
//                if (   (extracted_uuid.uuid == p_target_uuid->uuid)
//                    && (extracted_uuid.type == p_target_uuid->type))
//                {
//                    return true;
//                }
//            }
//        }
//        index += field_length + 1;
//    }
//    return false;
//}

///**@brief Function for handling events from the GATT library. */
//void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
//{
//    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
//    {
//        NRF_LOG_INFO("ATT MTU exchange completed.\r\n");
//
//        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
//        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)\r\n", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
//    }
//}

///**@brief Function for initializing the GATT library. */
//void gatt_init(void)
//{
//    ret_code_t err_code;
//
//    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
//    APP_ERROR_CHECK(err_code);
//
//    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_BLE_GATT_MAX_MTU_SIZE);
//    APP_ERROR_CHECK(err_code);
//}

///** @brief Function for initializing the Database Discovery Module.
// */
//static void db_discovery_init(void)
//{
//    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
//    APP_ERROR_CHECK(err_code);
//}

///**@brief Function for initializing the UART. */
//static void uart_init(void)
//{
//    ret_code_t err_code;
//
//    const app_uart_comm_params_t comm_params =
//      {
//        .rx_pin_no    = RX_PIN_NUMBER,
//        .tx_pin_no    = TX_PIN_NUMBER,
//        .rts_pin_no   = RTS_PIN_NUMBER,
//        .cts_pin_no   = CTS_PIN_NUMBER,
//        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
//        .use_parity   = false,
//        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
//      };
//
//    APP_UART_FIFO_INIT(&comm_params,
//                       UART_RX_BUF_SIZE,
//                       UART_TX_BUF_SIZE,
//                       uart_event_handle,
//                       APP_IRQ_PRIORITY_LOWEST,
//                       err_code);
//
//    APP_ERROR_CHECK(err_code);
//}
//
///**@brief Function for initializing the NUS Client. */
//static void nus_c_init(void)
//{
//    ret_code_t       err_code;
//    ble_nus_c_init_t init;
//
//    init.evt_handler = ble_nus_c_evt_handler;
//
//    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
//    APP_ERROR_CHECK(err_code);
//}

