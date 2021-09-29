#include "../include/stm32f446re_i2c_driver.h"

void I2C_PeriClockControl(I2C_regDef_t* pI2Cx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
    {
        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_EN();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_EN();
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_PCLK_EN();
        }
    }
    else
    {
        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_DI();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_DI();
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_PCLK_DI();
        }
    }
}

void I2C_PeripheralControl(I2C_regDef_t *pI2Cx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
    {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE_BIT);
    }
    else 
    {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE_BIT);
    }
}

void I2C_Init(I2C_Handle_t* pHandle)
{
    //1 configure the mode (standard or fast)
    //2 configure the speed of the serial clock (SCL)
    //3 configure the device address (applicable when the device is slave)
    //4 enable the Acking.
    //5 configure the rise time fo the i2C pins
    uint32_t tempreg = 0;
    // CR1 CALCULATIONS
    tempreg |= (pHandle->I2C_Config.ACK_Control << I2C_CR1_ACK_BIT);
    pHandle->pI2Cx->CR1 |= tempreg;

    tempreg = 0;
    
    // CR2 CALCULATIONS
    tempreg |= RCC_getPCLK1Value() / 1000000U;

    pHandle->pI2Cx->CR2 = (tempreg & 0x3F); //FREQ bit field.

    tempreg |= pHandle->I2C_Config.DeviceAddress << 1;
    tempreg |= (1 << 14); // Bit 14 should always be kept at 1 by software (per reference manual). Reason not stated.
    pHandle->pI2Cx->OAR1 = tempreg;

    tempreg = 0;
    //CCR CALCULATIONS
    uint16_t ccr_value = 0;

    if (pHandle->I2C_Config.SCL_Speed <= I2C_SCL_SPEED_SM)
    {
        //CCR mode is standard mode
        // pHandle->pI2Cx->CCR = RCC_getPCLK1Value() / (2 * pHandle->I2C_Config.SCL_Speed);
        ccr_value = RCC_getPCLK1Value() / (2 * pHandle->I2C_Config.SCL_Speed);
        tempreg |= (ccr_value & 0xFFF); // CCR FIRST 12 BITS ONLY
    }

    else
    {
        //mode is fast mode
        tempreg |= (1 << I2C_CCR_FS_BIT);
        tempreg |= (pHandle->I2C_Config.FM_DutyCycle << I2C_CCR_DUTY_BIT);

        if (pHandle->I2C_Config.FM_DutyCycle == I2C_FM_DUTY_2)
        {
            ccr_value = (RCC_getPCLK1Value() / (3 * pHandle->I2C_Config.SCL_Speed));
        }
        else
        {
            ccr_value = (RCC_getPCLK1Value() / (25 * pHandle->I2C_Config.SCL_Speed));
        }
        tempreg |= (ccr_value & 0xFFF); // CCR FIRST 12 BITS ONLY
    }
    pHandle->pI2Cx->CCR = tempreg;

    // TODO: TRISE CALCULATIONS


}

uint32_t RCC_GetPLLOutputClock(void)
{
    return;
}

/**
 *  0xxx: system clock not divided
    1000: system clock divided by 2
    1001: system clock divided by 4
    1010: system clock divided by 8
    1011: system clock divided by 16
    1100: system clock divided by 64
    1101: system clock divided by 128
    1110: system clock divided by 256
    1111: system clock divided by 512
*/

const uint16_t AHB_Prescaler[] = {2, 4, 8, 16, 64, 128, 256, 512};
const uint8_t APB_Prescalar[] = {2, 4, 8, 16};

uint32_t RCC_getPCLK1Value(void)
{
    uint32_t pclk1, SystemClk;
    uint8_t clksrc, temp, ahbp, apb1p;

    clksrc = ((RCC->CFGR >> 2) & 0x3);
    if (clksrc == 0)
    {
        SystemClk = 16000000;
    }
    else if (clksrc == 1)
    {
        SystemClk = 8000000;
    }
    else if (clksrc == 2)
    {
        SystemClk = RCC_GetPLLOutputClock();
    }

    temp = ((RCC ->CFGR >> 4) & 0x0F);

    if (temp < 8)
    {
        ahbp = 1;
    }
    else 
    {
        ahbp = AHB_Prescaler[temp - 8];
    }

    temp = ((RCC ->CFGR >> 10) & 0x07);
    if (temp < 4)
    {
        apb1p = 1;
    }
    else 
    {
        apb1p = APB_Prescalar[temp - 4];
    }
    pclk1 = (SystemClk / ahbp) / apb1p;


    return pclk1;
}

/**
 * typedef struct 
{
    uint32_t SCL_Speed;
    uint8_t DeviceAddress;
    uint8_t ACK_Control;
    uint8_t FM_DutyCycle;
} I2C_Config_t;

typedef struct 
{
    I2C_regDef_t *pI2Cx;
    I2C_Config_t I2C_Config;
} I2C_Handle_t;


 */ 