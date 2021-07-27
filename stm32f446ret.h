#ifndef STM32F446RET_H
#define STM32F446RET_H

#include <stdint.h>

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

/**
 * Base addresses of flash and sram memories.
 * 
 */

#define FLASH_BASEADDR 0x08000000U
#define SRAM1_BASEADDR 0X20000000U
#define SRAM2_BASEADDR 0X2001C000U
#define ROM_BASEADDR 0x1FFF0000U

/**
 * Peripheral buses
 * 
 */

#define AHB1_PERIPH_BASE 0x40020000U
#define AHB2_PERIPH_BASE 0x40023000U
#define AHB3_PERIPH_BASE 0xA0001000U

#define APB1_PERIPH_BASE 0x40000000U
#define APB2_PERIPH_BASE 0x40010000U

#define PERIPH_BASE APB1_PERIPH_BASE

/**
 * GPIO base addresses (AHB1 peripherals)
 * 
 */

#define GPIOA_BASEADDR (AHB1_PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR (AHB1_PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR (AHB1_PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR (AHB1_PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR (AHB1_PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR (AHB1_PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR (AHB1_PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR (AHB1_PERIPH_BASE + 0x1C00)

/**
 * APB1 peripherals
 * 
 */

#define TIM2_BASEADDR (APB1_PERIPH_BASE + 0x0000)
#define TIM3_BASEADDR (APB1_PERIPH_BASE + 0x0400)
#define TIM4_BASEADDR (APB1_PERIPH_BASE + 0x0800)
#define TIM5_BASEADDR (APB1_PERIPH_BASE + 0x0C00)
#define TIM6_BASEADDR (APB1_PERIPH_BASE + 0x1000)
#define TIM7_BASEADDR (APB1_PERIPH_BASE + 0x1400)
#define TIM12_BASEADDR (APB1_PERIPH_BASE + 0x1800)
#define TIM13_BASEADDR (APB1_PERIPH_BASE + 0x1C00)
#define TIM14_BASEADDR (APB1_PERIPH_BASE + 0x2000)
#define RTC_BKP_BASEADDR (APB1_PERIPH_BASE + 0x2800)
#define WWDG_BASEADDR (APB1_PERIPH_BASE + 0x2C00)
#define IWDG_BASEADDR (APB1_PERIPH_BASE + 0x3000)
#define SPI2_BASEADDR (APB1_PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR (APB1_PERIPH_BASE + 0x3C00)
#define SPDIF_RX_BASEADDR (APB1_PERIPH_BASE + 0x4000)
#define USART2_BASEADDR (APB1_PERIPH_BASE + 0x4400)
#define USART3_BASEADDR (APB1_PERIPH_BASE + 0x4800)
#define UART4_BASEADDR (APB1_PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR (APB1_PERIPH_BASE + 0x5000)
#define I2C1_BASEADDR (APB1_PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR (APB1_PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR (APB1_PERIPH_BASE + 0x5C00)
#define CAN1_BASEADDR (APB1_PERIPH_BASE + 0x6400)
#define CAN2_BASEADDR (APB1_PERIPH_BASE + 0x6800)
#define HDMI_CEC_BASEADDR (APB1_PERIPH_BASE + 0x6C00)
#define PWR_BASEADDR (APB1_PERIPH_BASE + 0x7000)
#define DAC_BASEADDR (APB1_PERIPH_BASE + 0x7400)

/**
 * AHB1 peripherals
 * 
 */

#define CRC_BASEADDR (AHB1_PERIPH_BASE + 0x3000)
#define RCC_BASEADDR (AHB1_PERIPH_BASE + 0x3800)
#define FLASH_INTERFACE_REG_BASEADDR (AHB1_PERIPH_BASE + 0x3C00)
#define BKPSRAM_BASEADDR (AHB1_PERIPH_BASE + 0x4000)
#define DMA1_BASEADDR (AHB1_PERIPH_BASE + 0x6000)
#define DMA2_BASEADDR (AHB1_PERIPH_BASE + 0x6400)
//#define DMA2_BASEADDR                   (AHB1_PERIPH_BASE + 0x00020000)

/**
 * APB2 peripherals 
 * 
 */
#define TIM1_BASEADDR (APB2_PERIPH_BASE)
#define TIM8_BASEADDR (APB2_PERIPH_BASE + 0x0400)
#define USART1_BASEADDR (APB2_PERIPH_BASE + 0x1000)
#define USART6_BASEADDR (APB2_PERIPH_BASE + 0x1400)
#define ADC1_BASEADDR (APB2_PERIPH_BASE + 0x2000)
#define ADC2_BASEADDR (APB2_PERIPH_BASE + 0x2000)
#define ADC3_BASEADDR (APB2_PERIPH_BASE + 0x2000)
#define SDMMC_BASEADDR (APB2_PERIPH_BASE + 0x2C00)
#define SPI1_BASEADDR (APB2_PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR (APB2_PERIPH_BASE + 0x3400)
#define SYSCFG_BASEADDR (APB2_PERIPH_BASE + 0x3800)
#define EXTI_BASEADDR (APB2_PERIPH_BASE + 0x3C00)
#define TIM9_BASEADDR (APB2_PERIPH_BASE + 0x4000)
#define TIM10_BASEADDR (APB2_PERIPH_BASE + 0x4400)
#define TIM11_BASEADDR (APB2_PERIPH_BASE + 0x4800)
#define SAI1_BASEADDR (APB2_PERIPH_BASE + 0x5800)
#define SAI2_BASEADDR (APB2_PERIPH_BASE + 0x5C00)

/**
 * Peripheral register defintion structures
 * 
 */

typedef struct
{
  volatile uint32_t MODER;
  volatile uint32_t OTYPER;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];

} GPIO_regDef_t;

/**
 * peripheral register structure for RCC.
 * 
 */

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t PLLCFGR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t AHB1RSTR;
  volatile uint32_t AHB2RSTR;
  volatile uint32_t AHB3RSTR;
  volatile uint32_t Reserved1;
  volatile uint32_t APB1RSTR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t Reserved2;
  volatile uint32_t Reserved3;
  volatile uint32_t AHB1ENR;
  volatile uint32_t AHB2ENR;
  volatile uint32_t AHB3ENR;
  volatile uint32_t Reserved4;
  volatile uint32_t APB1ENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t Reserved5;
  volatile uint32_t Reserved6;
  volatile uint32_t AHB1LPENR;
  volatile uint32_t AHB2LPENR;
  volatile uint32_t AHB3LPENR;
  volatile uint32_t Reserved7;
  volatile uint32_t APB1LPENR;
  volatile uint32_t APB2LPENR;
  volatile uint32_t Reserved8;
  volatile uint32_t Reserved9;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
  volatile uint32_t Reserved10;
  volatile uint32_t Reserved11;
  volatile uint32_t SSCGR;
  volatile uint32_t PLLI2SCFGR;
  volatile uint32_t PLLSAICFGR;
  volatile uint32_t DCKCFGR;
  volatile uint32_t CKGATENR;
  volatile uint32_t DCKCFGR2;

} RCC_regDef_t;

/**
 * Peripheral register structure for EXTI.
 * 
 */

typedef struct
{
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} EXTI_RegDef_t;

/**
 * Peripheral register structure for SYSCFG
 * 
 */

typedef struct
{
  volatile uint32_t MEMRMP;
  volatile uint32_t PMC;
  volatile uint32_t EXTICR[4];
  volatile uint32_t RESERVED1[2];
  volatile uint32_t CMPCR;
  volatile uint32_t RESERVED2[2];
  volatile uint32_t CFGR;
} SYSCFG_RegDef_t;

#define GPIOA ((GPIO_regDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_regDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_regDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_regDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_regDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_regDef_t *)GPIOF_BASEADDR)
#define GPIOG ((GPIO_regDef_t *)GPIOG_BASEADDR)
#define GPIOH ((GPIO_regDef_t *)GPIOH_BASEADDR)

#define RCC ((RCC_regDef_t *)RCC_BASEADDR)

#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)
// CLOCK ENABLE MACROS FOR GPIOx PERIPHERALS

#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))

// CLOCK ENABLE MACROS FOR I2Cx PERIPHERALS
#define I2C1_PCLK_EN() (RCC->ABP1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->ABP1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->ABP1ENR |= (1 << 23))

// CLOCK ENABLE MACROS FOR SPIx PERIPHERALS
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))

// CLOCK ENABLE MACROS FOR USARTx PERIPHERALS
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN() (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN() (RCC->APB1ENR |= (1 << 20))
#define USART6_PLCK_EN() (RCC->APB1ENR |= (1 << 5))

// CLOCK ENABLE MACROS FOR SYSCFG PERIPHERALS
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))

// CLOCK ENABLE MACROS FOR TIMER PERIPHERALS
#define TIM1_PCLK_EN() (RCC->APB2ENR |= (1 << 0))
#define TIM2_PCLK_EN() (RCC->APB1ENR |= (1 << 0))
#define TIM3_PCLK_EN() (RCC->APB1ENR |= (1 << 1))
#define TIM4_PCLK_EN() (RCC->APB1ENR |= (1 << 2))
#define TIM5_PCLK_EN() (RCC->APB1ENR |= (1 << 3))
#define TIM6_PCLK_EN() (RCC->APB1ENR |= (1 << 4))
#define TIM7_PCLK_EN() (RCC->APB1ENR |= (1 << 5))
#define TIM8_PCLK_EN() (RCC->APB2ENR |= (1 << 1))
#define TIM9_PCLK_EN() (RCC->APB2ENR |= (1 << 16))
#define TIM10_PCLK_EN() (RCC->APB2ENR |= (1 << 17))
#define TIM11_PCLK_EN() (RCC->APB2ENR |= (1 << 18))
#define TIM12_PCLK_EN() (RCC->APB1ENR |= (1 << 6))
#define TIM13_PCLK_EN() (RCC->APB1ENR |= (1 << 7))
#define TIM14_PCLK_EN() (RCC->APB1ENR |= (1 << 8))

// CLOCK DISABLE MACROS FOR GPIOx PERIPHERALS

#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 7))

// CLOCK DISABLE MACROS FOR I2Cx PERIPHERALS
#define I2C_PCLK_EN() (RCC->ABP1ENR &= ~(1 << 21))

// CLOCK DISABLE MACROS FOR SPIx PERIPHERALS

// CLOCK DISABLE MACROS FOR USARTx PERIPHERALS

// CLOCK DISABLE MACROS FOR SYSCFG PERIPHERALS

// MACROS TO RESET GPIO PERIPHERALS
#define GPIOA_REG_RESET()         \
  do                              \
  {                               \
    (RCC->AHB1RSTR |= (1 << 0));  \
    (RCC->AHB1RSTR &= ~(1 << 0)); \
  } while (0)
#define GPIOB_REG_RESET()         \
  do                              \
  {                               \
    (RCC->AHB1RSTR |= (1 << 1));  \
    (RCC->AHB1RSTR &= ~(1 << 1)); \
  } while (0)
#define GPIOC_REG_RESET()         \
  do                              \
  {                               \
    (RCC->AHB1RSTR |= (1 << 2));  \
    (RCC->AHB1RSTR &= ~(1 << 2)); \
  } while (0)
#define GPIOD_REG_RESET()         \
  do                              \
  {                               \
    (RCC->AHB1RSTR |= (1 << 3));  \
    (RCC->AHB1RSTR &= ~(1 << 3)); \
  } while (0)
#define GPIOE_REG_RESET()         \
  do                              \
  {                               \
    (RCC->AHB1RSTR |= (1 << 4));  \
    (RCC->AHB1RSTR &= ~(1 << 4)); \
  } while (0)
#define GPIOF_REG_RESET()         \
  do                              \
  {                               \
    (RCC->AHB1RSTR |= (1 << 5));  \
    (RCC->AHB1RSTR &= ~(1 << 5)); \
  } while (0)
#define GPIOG_REG_RESET()         \
  do                              \
  {                               \
    (RCC->AHB1RSTR |= (1 << 6));  \
    (RCC->AHB1RSTR &= ~(1 << 6)); \
  } while (0)
#define GPIOH_REG_RESET()         \
  do                              \
  {                               \
    (RCC->AHB1RSTR |= (1 << 7));  \
    (RCC->AHB1RSTR &= ~(1 << 7)); \
  } while (0)

#define GPIO_BASEADDR_TO_CODE(x) ((x == GPIOA) ? 0 : (x == GPIOB) ? 1 \
                                                 : (x == GPIOC)   ? 2 \
                                                 : (x == GPIOD)   ? 3 \
                                                 : (x == GPIOE)   ? 4 \
                                                 : (x == GPIOF)   ? 5 \
                                                 : (x == GPIOG)   ? 6 \
                                                 : (x == GPIOH)   ? 7 \
                                                                  : 0)

#endif
