/*
 * hw.h
 *
 *  Created on: Oct 6, 2020
 *      Author: Baram
 */

#ifndef SRC_HW_HW_H_
#define SRC_HW_HW_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "hw_def.h"


#include "led.h"
#include "uart.h"
#include "cmdif.h"


void hwInit(void);



#ifdef __cplusplus
}
#endif

#endif /* SRC_HW_HW_H_ */
