/*
 * hw_def.h
 *
 *  Created on: Oct 6, 2020
 *      Author: Baram
 */

#ifndef SRC_HW_HW_DEF_H_
#define SRC_HW_HW_DEF_H_


#include "def.h"
#include "bsp.h"


#define _USE_HW_FLASH



#define _USE_HW_LED
#define      HW_LED_MAX_CH          2

#define _USE_HW_UART
#define      HW_UART_MAX_CH         1

#define _USE_HW_CMDIF
#define      HW_CMDIF_LIST_MAX              32
#define      HW_CMDIF_CMD_STR_MAX           16
#define      HW_CMDIF_CMD_BUF_LENGTH        128

#define _USE_HW_QSPI
#define      HW_QSPI_DRIVER         MX25R6435F
#define      HW_QSPI_BASE_ADDR      QSPI_BASE


#endif /* SRC_HW_HW_DEF_H_ */
