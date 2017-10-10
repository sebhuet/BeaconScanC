/*
 * OtherTimer.cpp
 *
 *  Created on: 4 oct. 2017
 *      Author: seb
 */
#include "OtherTimer.h"

#if USEROTHERTIMER

#include <inttypes.h>
#include <stdbool.h>

#include "nrf_gpio.h"
#include "nrf.h"
#include "nrf52_bitfields.h"
#include "app_util_platform.h"
#include "softdevice_handler.h"

#define NRF_PPS_PIN     30
#define GPS_PPS_PIN     31

#define CC_MS           50

#define USE_SD

static volatile uint32_t _tick_ms;

void timer1_init(void)
{
    _tick_ms = 0;

    /* Configure Timer1 with 1us resolution */
    NRF_TIMER1->TASKS_STOP = 1;
    // Create an Event-Task shortcut to clear TIMER1 on COMPARE[0] event
    NRF_TIMER1->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
    NRF_TIMER1->PRESCALER = 4;  //1us resolution (HFCLK / 2^p, 16MHz / 2^4 = 1MHz)
    NRF_TIMER1->INTENSET = (TIMER_INTENSET_COMPARE0_Set << TIMER_INTENSET_COMPARE0_Pos);
    NRF_TIMER1->CC[0] = CC_MS * 1000;
    NRF_TIMER1->TASKS_START = 1;
    NVIC_SetPriority(TIMER1_IRQn, APP_IRQ_PRIORITY_HIGH);
    NVIC_EnableIRQ(TIMER1_IRQn);
}

void TIMER1_IRQHandler(void)
{
    _tick_ms += CC_MS;
    NRF_TIMER1->EVENTS_COMPARE[0] = 0;
    (void)NRF_TIMER1->EVENTS_COMPARE[0];
}

uint32_t timer1_uptime_ms(void)
{
    NRF_TIMER1->TASKS_CAPTURE[1] = 1;
    return _tick_ms + (NRF_TIMER1->CC[1] / 1000);
}

#endif //USEOTHERTIMER
