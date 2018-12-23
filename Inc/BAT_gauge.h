/*
 * BAT_gauge.h
 *
 *  Created on: 11 dec. 2018
 *      Author: cesar
 */


#ifndef BAT_GAUGE_H_
#define BAT_GAUGE_H_



#include <stdio.h>
#include <string.h>

#include "main.h"
#include "stm32l4xx_hal.h"

// Device specific values

#define GAUGE_ADDRESS 0xAA  // Write address.
#define MY_BAT_CAPACITY 1200  // [mAh]



I2C_HandleTypeDef hi2c1;

// Control() sub registers hex address

#define CONTROL_STATUS 0x0000
#define DEVICE_TYPE 0x0001
#define FW_VERSION 0x0002
#define DM_CODE 0x0004
#define PREV_MACWRITE 0x0007
#define CHEM_ID 0x0008
#define BAT_INSERT 0x000C
#define BAT_REMOVE 0x000D
#define SET_HIBERNATE 0x0011
#define CLEAR_HIBERNATE 0x0012
#define SET_CFGUPDATE 0x0013
#define SHUTDOWN_ENABLE 0x001B
#define SHUTDOWN 0x001C
#define SEALED 0x0020
#define TOGGLE_GPOUT 0x0023
#define RESET 0x0041
#define SOFT_RESET 0x0042
#define EXIT_CFGUPDATE 0x0043
#define EXIT_RESIM 0x0044



// Call functions

void Write_GAUGECMD( uint8_t * addressBuffer,  uint8_t * dataBuffer, uint8_t nBytes);

uint8_t Read_GAUGECMD(uint8_t * addressBuffer);

void Read_DataBlock(int offset, int nbits, uint8_t * returnData);

#endif /* BAT_GAUGE_H_ */
