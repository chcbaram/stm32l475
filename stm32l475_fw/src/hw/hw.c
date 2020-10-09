/*
 * hw.c
 *
 *  Created on: Oct 6, 2020
 *      Author: Baram
 */




#include "hw.h"





void hwInit(void)
{
  bspInit();


  cmdifInit();
  ledInit();
  uartInit();
  uartOpen(_DEF_UART1, 57600);
}

