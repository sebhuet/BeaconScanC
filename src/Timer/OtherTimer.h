/*
 * OtherTimer.h
 *
 *  Created on: 4 oct. 2017
 *      Author: seb
 */

#ifndef SRC_TIMER_OTHERTIMER_H_
#define SRC_TIMER_OTHERTIMER_H_

#define USEOTHERTIMER 0

#if USEOTHERTIMER
#include <inttypes.h>

extern void timer1_init(void);
extern void TIMER1_IRQHandler(void);
extern uint32_t timer1_uptime_ms(void);

#endif //USEOTHERTIMER
#endif /* SRC_TIMER_OTHERTIMER_H_ */
