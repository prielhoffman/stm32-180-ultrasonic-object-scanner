#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "stm32g0xx_hal.h"
#include <stdint.h>

/* Describes one LCD connected through an I2C backpack */
typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
} LCD_I2C_HandleTypeDef;

/*
 * Initializes the LCD driver and the physical display
 * address is the normal 7-bit I2C address, for example 0x27
 */
HAL_StatusTypeDef LCD_I2C_Init(LCD_I2C_HandleTypeDef *lcd, I2C_HandleTypeDef *hi2c, uint8_t address);

/* Clears all characters from the display */
HAL_StatusTypeDef LCD_I2C_Clear(LCD_I2C_HandleTypeDef *lcd);

/*
 * Moves the cursor
 * row:    0 or 1
 * column: 0 to 15
 */
HAL_StatusTypeDef LCD_I2C_SetCursor(LCD_I2C_HandleTypeDef *lcd, uint8_t row, uint8_t column);

/* Prints a null-terminated string from the current cursor position */
HAL_StatusTypeDef LCD_I2C_Print(LCD_I2C_HandleTypeDef *lcd, const char *text);

/*
 * Temporary diagnostic function
 * Turns the LCD backlight off and then on to verify the backpack pin mapping
 */
HAL_StatusTypeDef LCD_I2C_TestBacklight(LCD_I2C_HandleTypeDef *lcd);

#endif /* LCD_I2C_H */
