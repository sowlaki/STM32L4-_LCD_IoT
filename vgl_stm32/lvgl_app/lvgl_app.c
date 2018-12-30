/*
 * lvgl_app.c
 *
 *  Created on: 25 dec. 2018
 *      Author: cesar
 */


#include "lvgl_app.h"
#include "lvgl/lvgl.h"

/* Static variables  */


/* Call local functions */

void menu_init(void);

static lv_res_t btn_clicked_Start(lv_obj_t * btn);
static lv_res_t btn_pressed(lv_obj_t * btn);
static lv_res_t btn_clicked_Setting(lv_obj_t * btn);

void menu_start_init(void);
void menu_setting_init(void);

void lvgl_app_draw(void){

	menu_init();


}



void menu_init(void){

	lv_theme_t *theme = lv_theme_zen_init(210, lv_font_dejavu_10);

	lv_theme_set_current(theme);

	/* Create the background for the whole project */
	lv_obj_t * background = lv_obj_create(lv_scr_act(), NULL);
	lv_obj_set_size(background, 800, 480);
	lv_obj_set_pos(background, 0, 0);



	lv_obj_t * menu1 = lv_obj_create(background, NULL);
	lv_obj_set_size(menu1, 200, 200);
	lv_obj_align(menu1, NULL, LV_ALIGN_CENTER, 0 ,0);	// Put a box in the middle of the background


	/* Create button_start and button_setting, placed as children inside the menu1 arc. */
	lv_obj_t * button_start = lv_button_create(menu1, NULL);
	lv_obj_set_style(button_start, theme->btn.rel);
	lv_btn_set_fit(button_start, true, true);
	lv_obj_align(button_start, NULL, LV_ALIGN_IN_TOP_MID, 0, 40);

	lv_obj_t * button_setting = lv_button_create(menu1, NULL);
	lv_obj_set_style(button_setting, theme->btn.rel);
	lv_btn_set_fit(button_setting, true, true);
	lv_obj_align(button_setting, NULL, LV_ALIGN_CENTER, 0, 0);

	/* Add two labels to the buttons */
	lv_obj_t * label_start = lv_label_create(button_start, NULL);
	lv_label_set_text(label_start, "Start");

	lv_obj_t * label_setting = lv_label_create(button_setting, NULL);
	lv_label_set_text(label_setting, "Settings");

	/* Set actions for each button */

	lv_btn_set_action(button_start, LV_BTN_ACTION_CLICK,btn_clicked_Start);
	lv_btn_set_action(button_start, LV_BTN_ACTION_PR,btn_pressed);

	lv_btn_set_action(button_setting, LV_BTN_ACTION_CLICK, btn_clicked_Setting);
	lv_btn_set_action(button_setting, LV_BTN_ACTION_PR, btn_pressed);
}


static lv_res_t btn_clicked_Start(lv_obj_t * btn){

	lv_theme_t *theme = lv_theme_zen_init(210, lv_font_dejavu_10);
	lv_theme_set_current(theme);

	lv_obj_set_style(btn, theme->btn.pr);
	HAL_Delay(5);
	lv_obj_del(btn);

	menu_start_init();


	return 0;
}

static lv_res_t btn_clicked_Setting(lv_obj_t * btn){

	lv_theme_t *theme = lv_theme_zen_init(210, lv_font_dejavu_10);
	lv_theme_set_current(theme);

	lv_obj_set_style(btn, theme->btn.pr);
	HAL_Delay(5);
	lv_obj_del(btn);

	menu_setting_init();

	return 0;
}

static lv_res_t btn_pressed(lv_obj_t * btn){

	lv_theme_t *theme = lv_theme_zen_init(210, lv_font_dejavu_10);
	lv_theme_set_current(theme);

	lv_obj_set_style(btn, theme->btn.pr);

	return LV_RES_OK;
}



void menu_start_init(void){



}

void menu_setting_init(void){

}


