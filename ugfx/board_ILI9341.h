/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.io/license.html
 */

#ifndef _GDISP_LLD_BOARD_H
#define _GDISP_LLD_BOARD_H  
#include "main.h"  

extern SPI_HandleTypeDef hspi1;

static GFXINLINE void init_board(GDisplay *g) {
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static GFXINLINE void post_init_board(GDisplay *g) {
	(void) g;
}

static GFXINLINE void setpin_reset(GDisplay *g, gBool state) {
  HAL_GPIO_WritePin(lcd_reset_GPIO_Port, lcd_reset_Pin, state);
}

static GFXINLINE void set_backlight(GDisplay *g, gU8 percent) {
    if(percent > 0)
            HAL_GPIO_WritePin(lcd_blk_GPIO_Port, lcd_blk_Pin, SET);
    else
            HAL_GPIO_WritePin(lcd_blk_GPIO_Port, lcd_blk_Pin, RESET);
}

static GFXINLINE void acquire_bus(GDisplay *g) {
	(void) g;
}

static GFXINLINE void release_bus(GDisplay *g) {
	(void) g;
}

static GFXINLINE void write_index(GDisplay *g, gU16 index) {
  uint8_t tx_data;
  tx_data = index&0xff;
 
  HAL_GPIO_WritePin(GPIOC, lcd_dc_Pin, GPIO_PIN_RESET); 
  HAL_SPI_Transmit(&hspi1,&tx_data,1,100);
}

static GFXINLINE void write_data(GDisplay *g, gU16 data) {
  uint8_t tx_data;
  tx_data = data&0xff;
  
  HAL_GPIO_WritePin(GPIOC, lcd_dc_Pin, GPIO_PIN_SET); 
  HAL_SPI_Transmit(&hspi1,&tx_data,1,100);
}

static GFXINLINE void setreadmode(GDisplay *g) {
	(void) g;
}

static GFXINLINE void setwritemode(GDisplay *g) {
	(void) g;
}

static GFXINLINE gU16 read_data(GDisplay *g) {
  uint8_t rx_data[8];
  HAL_GPIO_WritePin(GPIOC, lcd_dc_Pin, GPIO_PIN_RESET); 
  HAL_StatusTypeDef status = HAL_SPI_Receive(&hspi1,rx_data,5,1300);
  return rx_data[0];
}

#endif /* _GDISP_LLD_BOARD_H */
