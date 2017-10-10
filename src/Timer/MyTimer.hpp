/*
 * MyTimer.hpp
 *
 *  Created on: 11 sept. 2017
 *      Author: seb
 */

#ifndef SRC_TIMER_MYTIMER_HPP_
#define SRC_TIMER_MYTIMER_HPP_

extern "C" {
	#include <stdint.h>
}

#define MYTIMERTICKS_US	10L

class MyTimer {

public:
	/** @brief IrHandler Singleton */
	static MyTimer &Get() {
		static MyTimer instance{};
		return instance;
	}

	__inline__ void ResetEllapsed()
	{
		_ellapsed = 0;
	}

	__inline__ void IncEllapsed()
	{
		_ellapsed++;
	}

	__inline__ uint64_t EllapsedMs()
	{
		return _ellapsed * MYTIMERTICKS_US / 1000UL;
	}

	__inline__ uint64_t EllapsedUs()
	{
		return _ellapsed * MYTIMERTICKS_US;
	}

	void _Setup();
	void _Start();
	void _Stop();

private:
	MyTimer();

private:
	uint64_t _ellapsed;

};

#endif /* SRC_TIMER_MYTIMER_HPP_ */
