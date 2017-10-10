/*
 * MyTimer.cpp
 *
 *  Created on: 11 sept. 2017
 *      Author: seb
 */
extern "C" {
#include "nrf_drv_timer.h"
#include "nrf_log.h"
//
#include "bsp.h"
}

#include "MyTimer.hpp"

const nrf_drv_timer_t _TIMER_MS = NRF_DRV_TIMER_INSTANCE(4);

static void inc_handler(nrf_timer_event_t event_type, void* p_context) {
	switch (event_type) {
	case NRF_TIMER_EVENT_COMPARE0:
		MyTimer::Get().IncEllapsed();
		break;

	default:
		//Do nothing.
		break;
	}
}

MyTimer::MyTimer() :
		_ellapsed(0)
{
	_Setup();
	_Start();
}

void MyTimer::_Setup() {
	static const nrf_drv_timer_config_t timer_config {
	    /*.frequency */ (nrf_timer_frequency_t)TIMER_DEFAULT_CONFIG_FREQUENCY,
	    /*.mode      */ (nrf_timer_mode_t)TIMER_DEFAULT_CONFIG_MODE,
	    /*.bit_width */ (nrf_timer_bit_width_t)TIMER_DEFAULT_CONFIG_BIT_WIDTH,
	    /*.interrupt_priority */ TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,                    \
	    /*.p_context          */ NULL                                                  \
	};
	const ret_code_t err_code = nrf_drv_timer_init(&_TIMER_MS, &timer_config, inc_handler);
	APP_ERROR_CHECK(err_code);
}

void MyTimer::_Start() {
	static const uint32_t time_ticks = nrf_drv_timer_us_to_ticks(&_TIMER_MS, MYTIMERTICKS_US);
	nrf_drv_timer_extended_compare(&_TIMER_MS, NRF_TIMER_CC_CHANNEL0,
			time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
	nrf_drv_timer_enable(&_TIMER_MS);
}

void MyTimer::_Stop() {
	nrf_drv_timer_compare_int_disable(&_TIMER_MS, NRF_TIMER_CC_CHANNEL0);
}
