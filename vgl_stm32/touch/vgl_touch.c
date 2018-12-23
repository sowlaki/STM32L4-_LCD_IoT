/*
 * vgl_touch.c
 *
 *  Created on: 15 okt. 2018
 *      Author: cesar
 */


#include "vgl_touch.h"
#include "touch.h"

#include "../lv_conf.h"
#include "../lvgl/lv_core/lv_vdb.h"
#include "../lvgl/lv_hal/lv_hal.h"




static bool read_touch_xy(lv_indev_data_t *data);


void lv_touch_init(void){


lv_indev_drv_t indev_drv;

lv_indev_drv_init(&indev_drv);

indev_drv.type = LV_INDEV_TYPE_POINTER;

indev_drv.read = read_touch_xy;

lv_indev_drv_register(&indev_drv);
}

static bool read_touch_xy(lv_indev_data_t *data) {

	/* static int16_t last_x = 0;
	static int16_t last_y = 0; */
	uint16_t TOUCH_X = 0;
	uint16_t TOUCH_Y = 0;

	TOUCH_X = CTS_read_X();
	TOUCH_Y = CTS_read_Y();

	data->point.x = TOUCH_X;
	data->point.y = TOUCH_Y;
	data->state = LV_INDEV_STATE_PR;

	return false;
}
