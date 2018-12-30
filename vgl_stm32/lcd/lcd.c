/*
 * lcd.c
 *
 *  Created on: 16 okt. 2018
 *      Author: cesar
 */

#include "main.h"
#include "lcd.h"
#include "stm32l4xx.h"
#include "stm32l4xx_hal.h"
#include "lvgl/lvgl.h"
#include "lv_conf.h"
#include "lvgl/lv_core/lv_vdb.h"
#include "lvgl/lv_hal/lv_hal.h"
#include "lvgl/lv_misc/lv_color.h"
#include <string.h>

/* DMA Stream parameters definitions. You can modify these parameters to select
   a different DMA Stream and/or channel.
   But note that only DMA2 Streams are capable of Memory to Memory transfers. */

#define DMA_STREAM_IRQHANDLER    DMA2_Stream0_IRQHandler
#define DMA_STREAM               DMA2_Stream0
#define DMA_CHANNEL              DMA2_CHANNEL
#define DMA_STREAM_IRQ           DMA2_Stream0_IRQ

static __IO uint32_t * my_fb = (__IO  uint8_t *) (OCTOSPI1_BASE);

/**********************
 *  STATIC VARIABLES
 **********************/


LTDC_HandleTypeDef hltdc;
DMA2D_HandleTypeDef hdma2d;

DMA_HandleTypeDef DmaHandle;
static int32_t x1_flush;
static int32_t y1_flush;
static int32_t x2_flush;
static int32_t y2_fill;
static int32_t y_fill_act;
static const lv_color_t * buf_to_flush;


/**********************
 *  STATIC PROTOTYPES
 **********************/

// DMA2D

static void DMA2D_Config();


static void DMA_TransferComplete(DMA_HandleTypeDef *han);
static void DMA_TransferError(DMA_HandleTypeDef *han);



// LVGL functions

static void tft_fill(int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t color);
static void tft_map(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p);
static void tft_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p);

static void gpu_mem_blend(lv_color_t * dest, const lv_color_t * src, uint32_t length, lv_opa_t opa);
static void gpu_mem_fill(lv_color_t * dest, uint32_t length, lv_color_t color);








 void lv_lcd_init(void){


	/* This is all for initializing functions and drivers for littlevgl */
	lv_disp_drv_t disp_drv;
	lv_disp_drv_init(&disp_drv);





	disp_drv.disp_fill = tft_fill;
	disp_drv.disp_map = tft_map;
	disp_drv.disp_flush = tft_flush;
	disp_drv.mem_blend = gpu_mem_blend;
	disp_drv.mem_fill = gpu_mem_fill;

	DMA2D_Config();

	lv_disp_drv_register(&disp_drv);
}



static void tft_fill(int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t color){

    /*Return if the area is out the screen*/
    if(x2 < 0) return;
    if(y2 < 0) return;
    if(x1 > TFT_HOR_RES - 1) return;
    if(y1 > TFT_VER_RES - 1) return;

    /*Truncate the area to the screen*/
    int32_t act_x1 = x1 < 0 ? 0 : x1;
    int32_t act_y1 = y1 < 0 ? 0 : y1;
    int32_t act_x2 = x2 > TFT_HOR_RES - 1 ? TFT_HOR_RES - 1 : x2;
    int32_t act_y2 = y2 > TFT_VER_RES - 1 ? TFT_VER_RES - 1 : y2;

	uint32_t x;
	uint32_t y;

	/*Fill the remaining area*/
	for(x = act_x1; x <= act_x2; x++) {
		for(y = act_y1; y <= act_y2; y++) {
			my_fb[y * TFT_HOR_RES + x] = color.full;
		}
	}

}

static void tft_map(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p){

	/*Return if the area is out the screen*/
		if(x2 < 0) return;
		if(y2 < 0) return;
		if(x1 > TFT_HOR_RES - 1) return;
		if(y1 > TFT_VER_RES - 1) return;

		/*Truncate the area to the screen*/
		int32_t act_x1 = x1 < 0 ? 0 : x1;
		int32_t act_y1 = y1 < 0 ? 0 : y1;
		int32_t act_x2 = x2 > TFT_HOR_RES - 1 ? TFT_HOR_RES - 1 : x2;
		int32_t act_y2 = y2 > TFT_VER_RES - 1 ? TFT_VER_RES - 1 : y2;

	#if LV_VDB_DOUBLE == 0
		uint32_t y;
		for(y = act_y1; y <= act_y2; y++) {
			memcpy((void*)&my_fb[y * TFT_HOR_RES + act_x1],
					color_p,
					(act_x2 - act_x1 + 1) * sizeof(my_fb[0]));
			color_p += x2 - x1 + 1;    /*Skip the parts out of the screen*/
		}
	#else

		x1_flush = act_x1;
		y1_flush = act_y1;
		x2_flush = act_x2;
		y2_fill = act_y2;
		y_fill_act = act_y1;
		buf_to_flush = color_p;


		  /*##-7- Start the DMA transfer using the interrupt mode #*/
		  /* Configure the source, destination and buffer size DMA fields and Start DMA Stream transfer */
		  /* Enable All the DMA interrupts */
		  if(HAL_DMA_Start_IT(&DmaHandle,(uint32_t)buf_to_flush, (uint32_t)&my_fb[y_fill_act * TFT_HOR_RES + x1_flush],
							  (x2_flush - x1_flush + 1)) != HAL_OK)
		  {
		    while(1)
		    {
		    }
		  }

	#endif


}

static void tft_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p){

	/*Return if the area is out the screen*/
		if(x2 < 0) return;
		if(y2 < 0) return;
		if(x1 > TFT_HOR_RES - 1) return;
		if(y1 > TFT_VER_RES - 1) return;

		/*Truncate the area to the screen*/
		int32_t act_x1 = x1 < 0 ? 0 : x1;
		int32_t act_y1 = y1 < 0 ? 0 : y1;
		int32_t act_x2 = x2 > TFT_HOR_RES - 1 ? TFT_HOR_RES - 1 : x2;
		int32_t act_y2 = y2 > TFT_VER_RES - 1 ? TFT_VER_RES - 1 : y2;

		x1_flush = act_x1;
		y1_flush = act_y1;
		x2_flush = act_x2;
		y2_fill = act_y2;
		y_fill_act = act_y1;
		buf_to_flush = color_p;


		  /*##-7- Start the DMA transfer using the interrupt mode #*/
		  /* Configure the source, destination and buffer size DMA fields and Start DMA Stream transfer */
		  /* Enable All the DMA interrupts */
		HAL_StatusTypeDef err;
		err = HAL_DMA_Start_IT(&DmaHandle,(uint32_t)buf_to_flush, (uint32_t)&my_fb[y_fill_act * TFT_HOR_RES + x1_flush],
				  (x2_flush - x1_flush + 1));
		if(err != HAL_OK)
		{
			while(1);	/*Halt on error*/
		}



}

static void gpu_mem_blend(lv_color_t * dest, const lv_color_t * src, uint32_t length, lv_opa_t opa){

	/*Wait for the previous operation*/
	HAL_DMA2D_PollForTransfer(&hdma2d, 100);
	hdma2d.Init.Mode         = DMA2D_M2M_BLEND;
	/* DMA2D Initialization */
	if(HAL_DMA2D_Init(&hdma2d) != HAL_OK)
	{
		/* Initialization Error */
		while(1);
	}

	hdma2d.LayerCfg[1].InputAlpha = opa;
    HAL_DMA2D_ConfigLayer(&hdma2d, 1);
	HAL_DMA2D_BlendingStart(&hdma2d, (uint32_t) src, (uint32_t) dest, (uint32_t)dest, length, 1);



}

static void gpu_mem_fill(lv_color_t * dest, uint32_t length, lv_color_t color){

	/*Wait for the previous operation*/
		HAL_DMA2D_PollForTransfer(&hdma2d, 100);

	   hdma2d.Init.Mode         = DMA2D_R2M;
	   /* DMA2D Initialization */
	   if(HAL_DMA2D_Init(&hdma2d) != HAL_OK)
	   {
	     /* Initialization Error */
	     while(1);
	   }


		hdma2d.LayerCfg[1].InputAlpha = 0xff;
	    HAL_DMA2D_ConfigLayer(&hdma2d, 1);
		// HAL_DMA2D_BlendingStart(&hdma2d, (uint32_t) lv_color_t(color), (uint32_t) dest, (uint32_t)dest, length, 1);

}

/*
 * 						*
 * LTDC CONFIGURATIONS	*
 * 						*
 */








/*
 * 						*
 * DMA2D CONFIGURATIONS	*
 * 						*
 */


static void DMA2D_TransferComplete(DMA2D_HandleTypeDef *hdma2d)
{

}

static void DMA2D_TransferError(DMA2D_HandleTypeDef *hdma2d)
{

}


static void DMA2D_Config(void){


	/* DMA2D Callbacks */
	hdma2d.XferCpltCallback  = DMA2D_TransferComplete;
	hdma2d.XferErrorCallback = DMA2D_TransferError;
}



/*
 * 						*
 * DMA CONFIGURATIONS	*
 * 						*
 */



static void DMA_TransferComplete(DMA_HandleTypeDef *han)
{
	y_fill_act ++;

	if(y_fill_act > y2_fill) {
		  lv_flush_ready();
	} else {
	  buf_to_flush += x2_flush - x1_flush + 1;
	  /*##-7- Start the DMA transfer using the interrupt mode ####################*/
	  /* Configure the source, destination and buffer size DMA fields and Start DMA Stream transfer */
	  /* Enable All the DMA interrupts */
	  if(HAL_DMA_Start_IT(han,(uint32_t)buf_to_flush, (uint32_t)&my_fb[y_fill_act * TFT_HOR_RES + x1_flush],
						  (x2_flush - x1_flush + 1)) != HAL_OK)
	  {
	    while(1);	/*Halt on error*/
	  }
	}
}

/**
  * @brief  DMA conversion error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
static void DMA_TransferError(DMA_HandleTypeDef *han)
{

}



/**
  * @brief  This function handles DMA Stream interrupt request.
  * @param  None
  * @retval None
  */
void DMA_STREAM_IRQHANDLER(void)
{
    /* Check the interrupt and clear flag */
    HAL_DMA_IRQHandler(&DmaHandle);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
