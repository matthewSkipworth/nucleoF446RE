#include "stm32f446ret_gpio_driver.h"



// GPIO CLOCK CONTROL

void GPIO_peripheral_clock_control(GPIO_regDef_t *pGPIOx, uint8_t ENorDI)
{
    if(ENorDI == ENABLE)
    {

        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_EN();
        }
    }
    else
    {
      
        if (pGPIOx == GPIOA)
        {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
            GPIOB_PCLK_DI();
        }
        else if (pGPIOx == GPIOC)
        {
            GPIOC_PCLK_DI();
        }
        else if (pGPIOx == GPIOD)
        {
            GPIOD_PCLK_DI();
        }
        else if (pGPIOx == GPIOE)
        {
            GPIOE_PCLK_DI();
        }
        else if (pGPIOx == GPIOF)
        {
            GPIOF_PCLK_DI();
        }
        else if (pGPIOx == GPIOG)
        {
            GPIOG_PCLK_DI();
        }
        else if (pGPIOx == GPIOH)
        {
            GPIOH_PCLK_DI();
        }
    }
}

// GPIO INITIALIZATION
void GPIO_init(GPIO_Handle_t *pGPIO_handle)
{
    uint32_t temp = 0;
    // 1. configure the mode of the gpio pin

    if (pGPIO_handle->GPIO_pinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        //non-interrupt mode

        temp = pGPIO_handle->GPIO_pinConfig.GPIO_PinMode << (2 * pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber);
        pGPIO_handle->pGPIOx->MODER &= ~(0x03 << pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber);
        pGPIO_handle->pGPIOx->MODER |= temp;
    }
    else
    {
        //interrupt mode
        //todo
    }
    temp = 0;
    
    // 2. configure the speed
    
    temp = pGPIO_handle->GPIO_pinConfig.GPIO_PinSpeed << (2 * pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber);
    pGPIO_handle->pGPIOx->OSPEEDR &= ~(0x03 << pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber);
    pGPIO_handle->pGPIOx->OSPEEDR |= temp;
    temp = 0;
    
    // 3. configure the pupd settings
    temp = pGPIO_handle->GPIO_pinConfig.GPIO_PinPuPdControl << (2 * pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber);
    pGPIO_handle->pGPIOx->PUPDR &= ~(0x03 << pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber);
    pGPIO_handle->pGPIOx->PUPDR |= temp;
    temp = 0;
    
    // 4. configure the otype settings
    temp = pGPIO_handle->GPIO_pinConfig.GPIO_PinOPType << (pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber);
    pGPIO_handle->pGPIOx->OTYPER &= ~(0x03 << pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber);
    pGPIO_handle->pGPIOx->OTYPER |= temp;
    temp = 0;
    
    // 5. configure the 
    if(pGPIO_handle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_ALTFUN)
    {
        uint32_t temp1, temp2;
        
        temp1 =  pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber % 8;
        
        pGPIO_handle->pGPIOx->AFR[temp1] &= (0xF << (4 * temp2));
        pGPIO_handle->pGPIOx->AFR[temp1] |= (pGPIO_handle->GPIO_pinConfig.GPIO_PinAltFunMode << (4 * temp2));
        
    }
    
}


//void GPIO_deinit(GPIO_regDef_t *pGPIOx)
//{

//}

//// GPIO READ/WRITE
//uint8_t GPIO_read_from_input_pin(GPIO_regDef_t *pGPIOx, uint8_t pin_number)
//{

//}
//uint16_t GPIO_read_from_input_port(GPIO_regDef_t *pGPIOx)
//{

//}
//void GPIO_write_to_output_pin(GPIO_regDef_t *pGPIOx, uint8_t pin_number, uint8_t value)
//{

//}
//void GPIO_write_to_output_port(GPIO_regDef_t *pGPIOx, uint16_t value)
//{

//}
//void GPIO_toggle_output_pin(GPIO_regDef_t *pGPIOx, uint8_t pin_number)
//{

//}

//// GPIO INTERRUPT HANDLING
//void GPIO_IRQ_config(uint8_t irq_number, uint8_t irq_priority, uint8_t ENorDI)
//{

//}
//void GPIO_IRQ_handling(uint8_t pin_number)
//{

//}

