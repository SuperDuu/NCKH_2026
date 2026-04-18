#include "ssd1306.h"

static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

static void SSD1306_WriteCommand(I2C_HandleTypeDef *hi2c, uint8_t cmd) {
    HAL_I2C_Mem_Write(hi2c, SSD1306_I2C_ADDR, 0x00, 1, &cmd, 1, 10);
}

void SSD1306_Init(I2C_HandleTypeDef *hi2c) {
    HAL_Delay(100);
    SSD1306_WriteCommand(hi2c, 0xAE); // Display Off
    SSD1306_WriteCommand(hi2c, 0x20); // Addressing Mode
    SSD1306_WriteCommand(hi2c, 0x00); // Horizontal
    SSD1306_WriteCommand(hi2c, 0xB0); // Page Start
    SSD1306_WriteCommand(hi2c, 0xC8); // COM Scan Normal
    SSD1306_WriteCommand(hi2c, 0x00); // Low column
    SSD1306_WriteCommand(hi2c, 0x10); // High column
    SSD1306_WriteCommand(hi2c, 0x40); // Start line
    SSD1306_WriteCommand(hi2c, 0x81); // Contrast
    SSD1306_WriteCommand(hi2c, 0xFF);
    SSD1306_WriteCommand(hi2c, 0xA1); // Seg Remap
    SSD1306_WriteCommand(hi2c, 0xA6); // Normal Display
    SSD1306_WriteCommand(hi2c, 0xA8); // Multiplex
    SSD1306_WriteCommand(hi2c, 0x3F);
    SSD1306_WriteCommand(hi2c, 0xA4); // Entire Display ON
    SSD1306_WriteCommand(hi2c, 0xD3); // Offset
    SSD1306_WriteCommand(hi2c, 0x00);
    SSD1306_WriteCommand(hi2c, 0xD5); // Clock Div
    SSD1306_WriteCommand(hi2c, 0xF0);
    SSD1306_WriteCommand(hi2c, 0xD9); // Pre-charge
    SSD1306_WriteCommand(hi2c, 0x22);
    SSD1306_WriteCommand(hi2c, 0xDA); // COM Pins
    SSD1306_WriteCommand(hi2c, 0x12);
    SSD1306_WriteCommand(hi2c, 0xDB); // VCOMH
    SSD1306_WriteCommand(hi2c, 0x20);
    SSD1306_WriteCommand(hi2c, 0x8D); // Charge Pump
    SSD1306_WriteCommand(hi2c, 0x14);
    SSD1306_WriteCommand(hi2c, 0xAF); // Display On
    SSD1306_Clear();
    SSD1306_UpdateScreen(hi2c);
}

void SSD1306_Clear(void) {
    for(uint16_t i=0; i < sizeof(SSD1306_Buffer); i++) SSD1306_Buffer[i] = 0x00;
}

void SSD1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color) {
    if(x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    if(color == White) SSD1306_Buffer[x + (y/8) * SSD1306_WIDTH] |= (1 << (y % 8));
    else               SSD1306_Buffer[x + (y/8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
}

// Vẽ đường thẳng (Bresenham) - bổ trợ cho FillRect/FillCircle
void SSD1306_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, SSD1306_COLOR color) {
    int16_t dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
    int16_t dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
    int16_t err = dx + dy, e2;
    while(1) {
        SSD1306_DrawPixel(x1, y1, color);
        if (x1 == x2 && y1 == y2) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x1 += sx; }
        if (e2 <= dx) { err += dx; y1 += sy; }
    }
}

void SSD1306_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, SSD1306_COLOR color) {
    for(uint8_t i = x; i < x + w; i++) {
        for(uint8_t j = y; j < y + h; j++) {
            SSD1306_DrawPixel(i, j, color);
        }
    }
}

void SSD1306_FillCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        SSD1306_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
        SSD1306_DrawLine(x0 - x, y0 - y, x0 + x, y0 - y, color);
        SSD1306_DrawLine(x0 - y, y0 + x, x0 + y, y0 + x, color);
        SSD1306_DrawLine(x0 - y, y0 - x, x0 + y, y0 - x, color);
    }
    SSD1306_DrawLine(x0 - r, y0, x0 + r, y0, color);
}

void SSD1306_UpdateScreen(I2C_HandleTypeDef *hi2c) {
    for(uint8_t i = 0; i < 8; i++) {
        SSD1306_WriteCommand(hi2c, 0xB0 + i);
        SSD1306_WriteCommand(hi2c, 0x00);
        SSD1306_WriteCommand(hi2c, 0x10);
        HAL_I2C_Mem_Write(hi2c, SSD1306_I2C_ADDR, 0x40, 1, &SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH, 100);
    }
}
