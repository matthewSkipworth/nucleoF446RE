#include "../include/stm32f446re.h"
#include "../include/stm32f446re_gpio_driver.h"
#include "../include/stm32f446re_spi_driver.h"

#include <stdio.h>
#include <string.h>

// pb15 can be spi2 mosi
// pb14 can be spi2 miso
// pb13 can be spi2 sclk
// pb12 can be spi2 nss
// Alternate function mode = 5

void SPI2_GPIO_Inits(void)
{
    GPIO_Handle_t SPI_Pins;
    SPI_Pins.pGPIOx = GPIOB;
    SPI_Pins.GPIO_pinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
    SPI_Pins.GPIO_pinConfig.GPIO_PinAltFunMode = 5;
    SPI_Pins.GPIO_pinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPI_Pins.GPIO_pinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPI_Pins.GPIO_pinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    
//    //NSS
    SPI_Pins.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_init(&SPI_Pins);
//    
    //SCLK
    SPI_Pins.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_init(&SPI_Pins);
    
//    //MISO
//    SPI_Pins.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//    GPIO_init(&SPI_Pins);
//    
    //MOSI
    SPI_Pins.GPIO_pinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_init(&SPI_Pins);
}

void SPI2_Inits(void)
{
    SPI_Handle_t SPI2_handle;
    SPI2_handle.pSPIx = SPI2;
    SPI2_handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI2_handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2_handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV_2; //8 MHz
    SPI2_handle.SPI_Config.SPI_DFF = SPI_DFF_8_BITS;
    SPI2_handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
    SPI2_handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
    SPI2_handle.SPI_Config.SPI_SSI = SPI_SSI_EN;
    SPI2_handle.SPI_Config.SPI_SSOE = SPI_SSOE_EN;
    SPI2_handle.SPI_Config.SPI_SSM = SPI_SSM_SW_EN;
    
//    SPI_PeriClockControl(SPI2_handle.pSPIx, ENABLE);
//    SPI2_PCLK_EN();
    SPI_Init(&SPI2_handle);
    
    
//    SPI_PeripheralControl(SPI2_handle.pSPIx, ENABLE);
}

int main(void)
{
    
    
    const char* hello = "Hello World!";

    SPI2_GPIO_Inits();
    SPI2_Inits();
    
//    SPI2->CR[0] |= (1 << SPI_CR1_SPE_BIT);
    
    SPI_PeripheralControl(SPI2, ENABLE);
    
    SPI_SendData(SPI2, (uint8_t *)0xFF, 1);
    SPI_PeripheralControl(SPI2, DISABLE);
    
    SPI_PeripheralControl(SPI2, ENABLE);
    SPI_SendData(SPI2, (uint8_t *)hello, strlen(hello));
    SPI_PeripheralControl(SPI2, DISABLE);
        
    
    while(1);
//    return 0;
}

