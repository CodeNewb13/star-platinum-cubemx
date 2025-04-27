/* ===== CH452_HAL.c ===== */
#include "CH452_HAL.h"

extern I2C_HandleTypeDef hi2c1;  // Ensure this is defined by MX_I2C1_Init()

// Low-level raw write to CH452 register
static HAL_StatusTypeDef CH452_WriteRaw(uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { (uint8_t)((reg & 0x0F) << 1) | 0x60, data };
    // The first byte sends command/address, second is payload
    return HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(CH452_I2C_ADDR << 1), buf, 2, HAL_MAX_DELAY);
}

// Low-level raw read from CH452 (for keyboard)
static HAL_StatusTypeDef CH452_ReadRaw(uint8_t *pData)
{
    uint8_t cmd = 0x6F;  // Read command for key scan
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(CH452_I2C_ADDR << 1), &cmd, 1, HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;
    return HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(CH452_I2C_ADDR << 1), pData, 1, HAL_MAX_DELAY);
}

void CH452_Init(void)
{
    // In CubeMX: Configure I2C1 on PB6=SCL, PB7=SDA, AF open-drain + hardware pull-ups
    // Ensure external pull-up resistors (4.7kO) on SCL and SDA lines

    // Give the CH452 a moment to power up
    HAL_Delay(5);

    // Enable I2C ACK and display/keyboard scanning
    CH452_WriteRaw(0x07, 0x00);  // ACK config
    HAL_Delay(1);
    CH452_WriteRaw(0x04, 0x03);  // Display + keyboard enable
    HAL_Delay(1);
}

void CH452_Clear(void)
{
    for (uint8_t i = 0; i < 8; i++) {
        CH452_WriteRaw(0x08 + i, 0x00);
    }
}

void CH452_SetDigit(uint8_t pos, uint8_t val)
{
    static const uint8_t bcd_table[16] = {
        0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,
        0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71
    };
    // Map position 0..7 to CH452 register 0x08+digit
    pos = (11 - pos) % 8;
    // Lower nibble is digit, high bit for decimal point
    uint8_t seg = bcd_table[val & 0x0F] | (val & 0x80);
    CH452_WriteRaw(0x08 + pos, seg);
}

void CH452_SetRam(uint8_t pos, uint8_t data)
{
    pos = (11 - pos) % 8;
    CH452_WriteRaw(0x08 + pos, data);
}

uint8_t CH452_GetKey(void)
{
    uint8_t code;
    if (CH452_ReadRaw(&code) != HAL_OK) return 16;
    switch (code) {
        case 0x4A: return 0;   case 0x47: return 1;
        case 0x46: return 2;   case 0x45: return 3;
        case 0x43: return 4;   case 0x42: return 5;
        case 0x41: return 6;   case 0x4F: return 7;
        case 0x4E: return 8;   case 0x4D: return 9;
        case 0x44: return 10;  case 0x40: return 11;
        case 0x4C: return 12;  case 0x48: return 13;
        case 0x4B: return 14;  case 0x49: return 15;
        default:    return 16;
    }
}