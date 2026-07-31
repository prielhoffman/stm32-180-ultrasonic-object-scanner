#include "lcd_i2c.h"

#define LCD_RS_BIT           (1U << 0)
#define LCD_ENABLE_BIT       (1U << 2)
#define LCD_BACKLIGHT_BIT    (1U << 3)
#define LCD_DATA_SHIFT       4U

#define LCD_FUNCTION_SET_4BIT_2LINE_5X8    0x28U
#define LCD_DISPLAY_OFF                    0x08U
#define LCD_CLEAR_DISPLAY                  0x01U
#define LCD_ENTRY_MODE_INCREMENT           0x06U
#define LCD_DISPLAY_ON_CURSOR_OFF          0x0CU


static HAL_StatusTypeDef LCD_I2C_WriteRaw(LCD_I2C_HandleTypeDef *lcd, uint8_t value){
    if ((lcd == NULL) || (lcd->hi2c == NULL)){
        return HAL_ERROR;
    }

    return HAL_I2C_Master_Transmit(lcd->hi2c, (uint16_t)(lcd->address << 1U), &value, 1U, 100U);
}


HAL_StatusTypeDef LCD_I2C_TestBacklight(LCD_I2C_HandleTypeDef *lcd){
    HAL_StatusTypeDef status;

    /* All PCF8574 outputs LOW. If P3 controls the backlight, it should turn off */
    status = LCD_I2C_WriteRaw(lcd, 0x00U);

    if (status != HAL_OK){
        return status;
    }

    HAL_Delay(500U);

    /* Set only P3 HIGH. If the assumed mapping is correct, the backlight should turn on again */
    status = LCD_I2C_WriteRaw(lcd, LCD_BACKLIGHT_BIT);

    return status;
}


static HAL_StatusTypeDef LCD_I2C_PulseEnable(LCD_I2C_HandleTypeDef *lcd, uint8_t value){
    HAL_StatusTypeDef status;

    status = LCD_I2C_WriteRaw(lcd, value | LCD_ENABLE_BIT);

    if (status != HAL_OK){
        return status;
    }

    HAL_Delay(1U);

    return LCD_I2C_WriteRaw(lcd, value & (uint8_t)(~LCD_ENABLE_BIT));
}


static HAL_StatusTypeDef LCD_I2C_WriteNibble(LCD_I2C_HandleTypeDef *lcd, uint8_t nibble, uint8_t rs){
    HAL_StatusTypeDef status;
    uint8_t value;

    /* Keep only the lowest four bits, then move them to PCF8574 pins P4-P7 */
    value = (uint8_t)((nibble & 0x0FU) << LCD_DATA_SHIFT);

    /* Keep the LCD backlight enabled */
    value |= LCD_BACKLIGHT_BIT;

    /*
     * RS = 1 for character data
     * RS = 0 for commands
     */
    if (rs != 0U){
        value |= LCD_RS_BIT;
    }

    /* First place the data on D4-D7 while E is LOW */
    status = LCD_I2C_WriteRaw(lcd, value);

    if (status != HAL_OK){
        return status;
    }

    /* Pulse E so the LCD captures the nibble  */
    return LCD_I2C_PulseEnable(lcd, value);
}


static HAL_StatusTypeDef LCD_I2C_SendByte(LCD_I2C_HandleTypeDef *lcd, uint8_t byte, uint8_t rs){
    HAL_StatusTypeDef status;
    uint8_t highNibble;
    uint8_t lowNibble;

    highNibble = (uint8_t)((byte >> 4U) & 0x0FU);
    lowNibble  = (uint8_t)(byte & 0x0FU);

    status = LCD_I2C_WriteNibble(lcd, highNibble, rs);

    if (status != HAL_OK){
        return status;
    }

    return LCD_I2C_WriteNibble(lcd, lowNibble, rs);
}


static HAL_StatusTypeDef LCD_I2C_SendCommand(LCD_I2C_HandleTypeDef *lcd, uint8_t command){
    return LCD_I2C_SendByte(lcd, command, 0U);
}


static HAL_StatusTypeDef LCD_I2C_SendData(LCD_I2C_HandleTypeDef *lcd, uint8_t data){
    return LCD_I2C_SendByte(lcd, data, 1U);
}

HAL_StatusTypeDef LCD_I2C_Init(LCD_I2C_HandleTypeDef *lcd, I2C_HandleTypeDef *hi2c, uint8_t address){
    HAL_StatusTypeDef status;

    /* Validate the pointers and the 7-bit I2C address */
    if ((lcd == NULL) || (hi2c == NULL) || (address > 0x7FU)){
        return HAL_ERROR;
    }

    /* Store the hardware information inside the LCD handle */
    lcd->hi2c = hi2c;
    lcd->address = address;

    /* The datasheet requires more than 15 ms after power-up, we use 50 ms to provide a safe margin */
    HAL_Delay(50U);

    /* Force the LCD interface into a known state, these are individual nibbles, not complete bytes */
    status = LCD_I2C_WriteNibble(lcd, 0x03U, 0U);

    if (status != HAL_OK){
        return status;
    }

    HAL_Delay(5U);

    status = LCD_I2C_WriteNibble(lcd, 0x03U, 0U);

    if (status != HAL_OK){
        return status;
    }

    HAL_Delay(1U);

    status = LCD_I2C_WriteNibble(lcd, 0x03U, 0U);

    if (status != HAL_OK){
        return status;
    }

    HAL_Delay(1U);

    /* 0010 on D7-D4 selects the 4-bit interface */
    status = LCD_I2C_WriteNibble(lcd, 0x02U, 0U);

    if (status != HAL_OK){
        return status;
    }

    HAL_Delay(1U);

    /* From this point onward, commands are full bytes: high nibble first, then low nibble */

    status = LCD_I2C_SendCommand(lcd, LCD_FUNCTION_SET_4BIT_2LINE_5X8);

    if (status != HAL_OK){
        return status;
    }

    status = LCD_I2C_SendCommand(lcd, LCD_DISPLAY_OFF);

    if (status != HAL_OK){
        return status;
    }

    status = LCD_I2C_SendCommand(lcd, LCD_CLEAR_DISPLAY);

    if (status != HAL_OK){
        return status;
    }

    /* Clear Display is slower than most LCD commands */
    HAL_Delay(2U);

    status = LCD_I2C_SendCommand(lcd, LCD_ENTRY_MODE_INCREMENT);

    if (status != HAL_OK){
        return status;
    }

    return LCD_I2C_SendCommand(lcd, LCD_DISPLAY_ON_CURSOR_OFF);
}


HAL_StatusTypeDef LCD_I2C_Print(LCD_I2C_HandleTypeDef *lcd, const char *text){
    HAL_StatusTypeDef status;

    if ((lcd == NULL) || (text == NULL)){
        return HAL_ERROR;
    }

    while (*text != '\0'){
        status = LCD_I2C_SendData(lcd, (uint8_t)(*text));

        if (status != HAL_OK){
            return status;
        }

        text++;
    }

    return HAL_OK;
}


HAL_StatusTypeDef LCD_I2C_SetCursor(LCD_I2C_HandleTypeDef *lcd, uint8_t row, uint8_t column){
    uint8_t ddramAddress;
    uint8_t command;

    if ((lcd == NULL) || (row > 1U) || (column > 15U)){
        return HAL_ERROR;
    }

    if (row == 0U){
        ddramAddress = column;
    }
    else{
        ddramAddress = (uint8_t)(0x40U + column);
    }

    command = (uint8_t)(0x80U | ddramAddress);

    return LCD_I2C_SendCommand(lcd, command);
}


HAL_StatusTypeDef LCD_I2C_Clear(LCD_I2C_HandleTypeDef *lcd){
    HAL_StatusTypeDef status;

    if (lcd == NULL){
        return HAL_ERROR;
    }

    status = LCD_I2C_SendCommand(lcd, LCD_CLEAR_DISPLAY);

    if (status != HAL_OK){
        return status;
    }

    /* Clear Display takes longer than most LCD commands */
    HAL_Delay(2U);

    return HAL_OK;
}
