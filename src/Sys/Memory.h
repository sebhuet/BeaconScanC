/*
 * Memory.hpp
 *
 *  Created on: 8 sept. 2017
 *      Author: seb
 */

#ifndef SRC_SYS_MEMORY_H_
#define SRC_SYS_MEMORY_H_


void* operator new(size_t size);
void operator delete(void* ptr, unsigned int);

#endif /* SRC_SYS_MEMORY_H_ */
