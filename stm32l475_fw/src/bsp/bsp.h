/*
 * bsp.h
 *
 *  Created on: Oct 6, 2020
 *      Author: Baram
 */

#ifndef SRC_BSP_BSP_H_
#define SRC_BSP_BSP_H_


#include "def.h"


#ifdef __cplusplus
extern "C" {
#endif


#include "stm32l4xx_hal.h"


void bspInit(void);


void delay(uint32_t delay_ms);
uint32_t millis(void);


void Error_Handler(void);


#ifdef __cplusplus
}
#endif


#endif /* SRC_BSP_BSP_H_ */
