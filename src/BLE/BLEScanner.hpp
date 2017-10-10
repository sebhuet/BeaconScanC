/*
 * BLEScanner.h
 *
 *  Created on: 8 sept. 2017
 *      Author: seb
 */

#ifndef SRC_BLE_BLESCANNER_HPP_
#define SRC_BLE_BLESCANNER_HPP_

extern "C" {

}

class BLEScanner {

public:

	/** @brief IrHandler Singleton */
	static BLEScanner &Get() {
		static BLEScanner instance{};
		return instance;
	}

	void Open() {
	    //db_discovery_init();
	    //gatt_init();
	    //nus_c_init();

	    // Start scanning for peripherals and initiate connection
	    // with devices that advertise NUS UUID.
	    // printf("BLE UART central example started.\r\n");
	}

	/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
	 *
	 * @details This function is called from the scheduler in the main loop after a BLE stack event has
	 *          been received.
	 *
	 * @param[in] p_ble_evt  Bluetooth stack event.
	 */
	void ble_evt_dispatch(ble_evt_t * p_ble_evt);

private:
	BLEScanner();

	/**@brief Function for initializing the BLE stack.
	 *
	 * @details Initializes the SoftDevice and the BLE event interrupt.
	 */
	void ble_stack_init(void);

	/**@brief Function to start scanning. */
	void scan_start(void);


	/**@brief Function for handling the Application's BLE Stack events.
	 *
	 * @param[in] p_ble_evt  Bluetooth stack event.
	 */
	void on_ble_evt(ble_evt_t * p_ble_evt);
};

extern BLEScanner *myBleScanner;

#endif /* SRC_BLE_BLESCANNER_HPP_ */
