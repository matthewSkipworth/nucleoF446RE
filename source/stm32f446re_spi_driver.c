#include "../include/stm32f446re_spi_driver.h"

//PERIPHERAL CLOCK CONTROL
void SPI_PeriClockControl(SPI_regDef_t *pSPIx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_PCLK_EN();
        }
    }
    else
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if (pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if (pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }
        else if (pSPIx == SPI4)
        {
            SPI4_PCLK_DI();
        }
    }
}

//INIT AND DE-INIT
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
    
    uint32_t tempReg2=0, tempReg = 0;
    // configure the device mode.
    tempReg |= (pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR_BIT);

    if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        //BIDI MODE SHOULD BE CLEARED
        tempReg &= ~(1 << SPI_CR1_BIDIMODE_BIT);

    }
    else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        //BIDI MODE SHOULD BE SET
        tempReg |= (1 << SPI_CR1_BIDIMODE_BIT);
    }
    else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_SIMPLEX_RXONLY)
    {
        //BIDI MODE SHOULD BE CLEARED;
        tempReg &= ~(1 << SPI_CR1_BIDIMODE_BIT);
        //RXONLY BIT MUST BE SET.
        tempReg |= (1 << SPI_CR1_RXONLY_BIT);
    }

    tempReg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR_BITS;
    
    tempReg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM_BIT;
    
    tempReg |= pSPIHandle->SPI_Config.SPI_SSI << SPI_CR1_SSI_BIT;
    
    tempReg2 |= pSPIHandle->SPI_Config.SPI_SSOE << SPI_CR2_SSOE_BIT;
    
    tempReg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF_BIT;

    tempReg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL_BIT;

    tempReg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA_BIT;

    pSPIHandle->pSPIx->CR1 = tempReg;
    pSPIHandle->pSPIx->CR2 = tempReg2;
}
void SPI_DeInit(SPI_regDef_t *pSPIx)
{
    // use rcc reset registers to reset the spi peripheral.
}

//SEND AND RECEIVE DATA
void SPI_SendData(SPI_regDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t length)
{
    while (length > 0)
    {
        uint16_t tempReg = 0;
        tempReg = pSPIx->DR;
        
        while(pSPIx->SR & (1 << SPI_SR_RXNE_BIT));
        while(!(pSPIx->SR & (1 << SPI_SR_TXE_BIT)));
        if (pSPIx->CR1 & (1 << SPI_CR1_DFF_BIT))
        {
            // load dr with 1 byte of data and increment the buffer address
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            (uint16_t*) pTxBuffer++; 
            length--;
            length--;
        }
        else
        {
            // load dr with 1 byte of data and increment the buffer address
            pSPIx->DR = *pTxBuffer;
            pTxBuffer++;
            length--;
        }
    
    }
}


void SPI_ReceiveData(SPI_regDef_t *pSPIx, uint8_t* pRxBuffer, uint32_t length )
{

}

//IRQ CONFIGURATION AND ISR HANDLING
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}
void SPI_IRQ_config(uint8_t irq_number/*, uint8_t irq_priority*/, uint8_t ENorDI)
{

}
void SPI_IRQ_handling(SPI_Handle_t* pHandle)
{

}

void SPI_PeripheralControl(SPI_regDef_t *pSPIx, uint8_t ENorDI)
{
    
    if (ENorDI == ENABLE)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE_BIT);
    }
    else
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE_BIT);
    }
    
    
}

//OTHER PERIPHERAL CONTROL APIS

