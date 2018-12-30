# Littlev Graphics Library

![LittlevGL cover](https://littlevgl.com/docs/themes/lv_theme_intro.png)




## Porting

 
## Project set-up
1. **Clone** or [Download](https://littlevgl.com/download) the lvgl repository: `git clone  https://github.com/littlevgl/lvgl.git`
2. **Create project** with your preferred IDE and add the *lvgl* folder
3. Copy **lvgl/lv_conf_templ.h** as **lv_conf.h** next to the *lvgl* folder
4. In the lv_conf.h delete the first `#if 0` and its `#endif`. Leave the default configuration for the first try.
5. In your *main.c*: #include "lvgl/lvgl.h"   
6. In your *main function*:
   * lvgl_init();
   * tick, display and input device initialization (see above)
7. To **test** create a label: `lv_obj_t * label = lv_label_create(lv_scr_act(), NULL);`  
8. In the main *while(1)* call `lv_task_handler();` and make a few milliseconds delay (e.g. `my_delay_ms(5);`) 
9. Compile the code and load it to your embedded hardware



## Related repositories


## Screenshots


#
