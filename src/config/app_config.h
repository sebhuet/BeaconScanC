/*
 * app_config.hpp
 *
 *  Created on: 4 oct. 2017
 *      Author: seb
 */
#ifndef SRC_CONFIG_APP_CONFIG_H_
#define SRC_CONFIG_APP_CONFIG_H_

#ifdef USE_APP_CONFIG

// <e> CLOCK_ENABLED - nrf_drv_clock - CLOCK peripheral driver
//==========================================================
#define CLOCK_ENABLED 1

// <e> GPIOTE_ENABLED - nrf_drv_gpiote - GPIOTE peripheral driver
//==========================================================
#define GPIOTE_ENABLED 1

// <e> TIMER_ENABLED - nrf_drv_timer - TIMER periperal driver
//==========================================================
#define TIMER_ENABLED 1

#if TIMER_ENABLED
#define TIMER0_ENABLED 1
#define TIMER1_ENABLED 1
#define TIMER2_ENABLED 1
#define TIMER3_ENABLED 1
#define TIMER4_ENABLED 1
#endif //TIMER_ENABLED

#endif

#define SEGGER_RTT_CONFIG_BUFFER_SIZE_UP 1024

#endif /* SRC_CONFIG_APP_CONFIG_H_ */
