/*
 * touch.h
 *
 *  Created on: 20 sep. 2018
 *      Author: cesar
 */



#ifndef TOUCH_H_
#define TOUCH_H_

#include "main.h"
#include "stm32l4xx_hal.h"




#define CTS_ADDRESS 0x70




// extern variables



int CTS_read_X(void);

int CTS_read_Y(void);



#endif /* TOUCH_H_ */
