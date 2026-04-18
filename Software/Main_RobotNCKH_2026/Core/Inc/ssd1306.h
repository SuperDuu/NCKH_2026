#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "stm32h7xx_hal.h"
#include <stdlib.h>

// Thông số màn hình
#define SSD1306_I2C_ADDR        (0x3C << 1)
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          64

// Màu sắc
typedef enum {
    Black = 0x00,
    White = 0x01
} SSD1306_COLOR;

// Prototypes
void SSD1306_Init(I2C_HandleTypeDef *hi2c);
void SSD1306_UpdateScreen(I2C_HandleTypeDef *hi2c);
void SSD1306_Clear(void);
void SSD1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color);
void SSD1306_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, SSD1306_COLOR color);
void SSD1306_FillCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR color);
void SSD1306_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, SSD1306_COLOR color);

#endif
