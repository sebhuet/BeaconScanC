/*
 * Memory.cpp
 *
 *  Created on: 8 sept. 2017
 *      Author: seb
 */
extern "C" {
	#include <stdlib.h> // for malloc and free
}

#include "Memory.h"

void* operator new(size_t size)
{
	return malloc(size);
}

void operator delete(void* ptr, unsigned int)
{
	if (ptr) free(ptr);
}



