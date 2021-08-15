#ifndef STM32F446RE_SPI_DRIVER
#define STM32F446RE_SPI_DRIVER

#include "stm32f446re.h"

/**
 *  @SPI_DeviceMode
 */

#define SPI_DEVICE_MODE_SLAVE               0
#define SPI_DEVICE_MODE_MASTER              1

/**
 * @SPI_BusConfig
 */ 

#define SPI_BUS_CONFIG_FD                   1
#define SPI_BUS_CONFIG_HD                   2
#define SPI_BUS_SIMPLEX_RXONLY              3

/**
 * @SPI_SclkSpeed
 */

#define SPI_SCLK_SPEED_DIV_2                0
#define SPI_SCLK_SPEED_DIV_4                1
#define SPI_SCLK_SPEED_DIV_8                2
#define SPI_SCLK_SPEED_DIV_16               3
#define SPI_SCLK_SPEED_DIV_32               4
#define SPI_SCLK_SPEED_DIV_64               5
#define SPI_SCLK_SPEED_DIV_128              6
#define SPI_SCLK_SPEED_DIV_256              7


/**
 * @SPI_DFF
 */ 

#define SPI_DFF_8_BITS                      0
#define SPI_DFF_16_BITS                     1

/**
 * @SPI_CPOL //CLOCK POLARITY
 */

#define SPI_CPOL_LOW                        0
#define SPI_CPOL_HIGH                       1

/**
 * @SPI_CPHA //CLOCK PHASE
 */

#define SPI_CPHA_LOW                        0
#define SPI_CPHA_HIGH                       1

/**
 * @SPI_SSM 
 */

#define SPI_SSM_SW_DI                       0
#define SPI_SSM_SW_EN                       1

/**
 * @SPI_SSI
 */

#define SPI_SSI_DI                          0
#define SPI_SSI_EN                          1

/**
 * @SPI_SSOE
 */

#define SPI_SSOE_DI                         0
#define SPI_SSOE_EN                         1

/**
 * @SPI_DEVICEMODE
 */

#define SPI_SLAVE_MODE                      0
#define SPI_MASTER_MODE                     1

/**
 * @CR1 BITS
 */
#define SPI_CR1_CPHA_BIT                    0
#define SPI_CR1_CPOL_BIT                    1
#define SPI_CR1_MSTR_BIT                    2
#define SPI_CR1_BR_BITS                     3 //BITS 3-5
#define SPI_CR1_SPE_BIT                     6
#define SPI_CR1_LSBFIRST_BIT                7
#define SPI_CR1_SSI_BIT                     8
#define SPI_CR1_SSM_BIT                     9
#define SPI_CR1_RXONLY_BIT                  10
#define SPI_CR1_DFF_BIT                     11
#define SPI_CR1_CRCNEXT_BIT                 12
#define SPI_CR1_CRCEN_BIT                   13
#define SPI_CR1_BIDIOE_BIT                  14
#define SPI_CR1_BIDIMODE_BIT                15

/**
 * @CR2 BITS
 */

#define SPI_CR2_RXDMAEN_BIT                 0
#define SPI_CR2_TXDMAEN_BIT                 1
#define SPI_CR2_SSOE_BIT                    2
#define SPI_CR2_FRF_BIT                     4
#define SPI_CR2_ERRIE_BIT                   5
#define SPI_CR2_RXNEIE_BIT                  6
#define SPI_CR2_TXEIE_BIT                   7

/**
 * @SR BITS
 */

#define SPI_SR_RXNE_BIT                     0
#define SPI_SR_TXE_BIT                      1
#define SPI_SR_CHISIDE_BIT                  2
#define SPI_SR_UDR_BIT                      3
#define SPI_SR_CRCERR_BIT                   4
#define SPI_SR_MODF_BIT                     5
#define SPI_SR_OVR_BIT                      6
#define SPI_SR_BSY_BIT                      7
#define SPI_SR_FRE_BIT                      8
/*
 * Configuration Structure for the SPIx peripheral
 */

typedef struct 
{
    /* data */
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
    uint8_t SPI_SSI;
    uint8_t SPI_SSOE;
} SPI_Config_t;


/*
 * Handle structure for the SPIx peripheral
 */

typedef struct 
{
    SPI_regDef_t *pSPIx;
    SPI_Config_t SPI_Config;
} SPI_Handle_t;

//PERIPHERAL CLOCK CONTROL
void SPI_PeriClockControl(SPI_regDef_t *pSPIx, uint8_t ENorDI);

//INIT AND DE-INIT
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_regDef_t *pSPIx);

//SEND AND RECEIVE DATA
void SPI_SendData(SPI_regDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t length);
void SPI_ReceiveData(SPI_regDef_t *pSPIx, uint8_t* pRxBuffer, uint32_t length );

//IRQ CONFIGURATION AND ISR HANDLING
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQ_config(uint8_t irq_number/*, uint8_t irq_priority*/, uint8_t ENorDI);
void SPI_IRQ_handling(SPI_Handle_t* pHandle);

//OTHER PERIPHERAL CONTROL APIS

void SPI_PeripheralControl(SPI_regDef_t *pSPIx, uint8_t ENorDI);

#endif //STM32F446RE_SPI_DRIVER

