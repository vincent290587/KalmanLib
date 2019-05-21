/*
 * segger_wrapper.h
 *
 *  Created on: 5 oct. 2017
 *      Author: Vincent
 */

#ifndef LIBRARIES_UTILS_SEGGER_WRAPPER_H_
#define LIBRARIES_UTILS_SEGGER_WRAPPER_H_

#include <stdint.h>

#define EMPTY_MACRO                    do {} while (0)

#define SYSTEM_DESCR_POS_CRC           0xFD


#ifdef TDD
#include "segger_wrapper_tdd.h"
#else
#include "segger_wrapper_arm.h"
#endif


#endif /* LIBRARIES_UTILS_SEGGER_WRAPPER_H_ */
