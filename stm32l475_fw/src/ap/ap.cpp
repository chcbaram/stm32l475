/*
 * ap.cpp
 *
 *  Created on: Oct 6, 2020
 *      Author: Baram
 */




#include "ap.h"





void apInit(void)
{
  hwInit();
}

void apMain(void)
{
  uint32_t pre_time[2];

  while(1)
  {
    if (millis()-pre_time[0] >= 500)
    {
      pre_time[0] = millis();
      ledToggle(_DEF_LED1);
      uartPrintf(_DEF_UART1, "test\n");
    }
    if (millis()-pre_time[1] >= 100)
    {
      pre_time[1] = millis();
      ledToggle(_DEF_LED2);
    }

    if (uartAvailable(_DEF_UART1) > 0)
    {
      uartPrintf(_DEF_UART1, "rx 0x%X\n", uartRead(_DEF_UART1));
    }
  }
}
