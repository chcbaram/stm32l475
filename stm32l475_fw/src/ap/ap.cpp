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


  cmdifOpen(_DEF_UART1, 57600);
}

void apMain(void)
{
  uint32_t pre_time[2];

  while(1)
  {
    cmdifMain();


    if (millis()-pre_time[0] >= 500)
    {
      pre_time[0] = millis();
      ledToggle(_DEF_LED1);
    }
    if (millis()-pre_time[1] >= 100)
    {
      pre_time[1] = millis();
      ledToggle(_DEF_LED2);
    }
  }
}
