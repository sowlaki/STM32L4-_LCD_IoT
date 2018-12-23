/*
 * lcd.h
 *
 *  Created on: 16 okt. 2018
 *      Author: cesar
 */

#ifndef LCD_H_
#define LCD_H_


#include <stdint.h>
#include "main.h"
#include "lvgl/lv_misc/lv_color.h"
#include "lvgl/lv_misc/lv_area.h"



/*********************
 *      DEFINES
 *********************/
#define TFT_HOR_RES 800
#define TFT_VER_RES 480

#define TFT_EXT_FB		0		/*Frame buffer is located into an external SDRAM*/
#define TFT_USE_GPU		1		/*Enable hardware accelerator*/

/**********************
 *      TYPEDEFS
 **********************/

uint32_t my_fb [1];

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void lv_lcd_init(void);
/**********************
 *      MACROS
 **********************/


#endif /* LCD_H_ */
