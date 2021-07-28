#include "stm32f446ret_gpio_driver.h"

// GPIO CLOCK CONTROL

void GPIO_peripheral_clock_control(GPIO_regDef_t *pGPIOx, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
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
        if (pGPIO_handle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_FE)
        {
            // configure the FTSR
            EXTI->FTSR |= (1 << pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber);
            EXTI->RTSR &= ~(1 << pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber);
        }
        else if (pGPIO_handle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_RE)
        {
            // configure the RTSR
            EXTI->RTSR |= (1 << pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber);
            EXTI->FTSR &= ~(1 << pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber);
        }
        else if (pGPIO_handle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_IT_FERE)
        {
            // cofigure both FTSR and RTSR
            EXTI->FTSR |= (1 << pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber);
            EXTI->RTSR |= (1 << pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber);
        }
        // configure the GPIO port select in SYSCFG_EXTICR
        uint8_t temp1 = pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber / 4;
        uint8_t temp2 = pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber % 4;
        uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIO_handle->pGPIOx);
        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[temp1] = portcode << temp2 * 4;

        // enable the exti interrupt delivery using the IMR (interrupt mask register)
        EXTI->IMR |= (1 << pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber);
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
    if (pGPIO_handle->GPIO_pinConfig.GPIO_PinMode == GPIO_MODE_ALTFUN)
    {
        uint32_t temp1, temp2;

        temp1 = pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber / 8;
        temp2 = pGPIO_handle->GPIO_pinConfig.GPIO_PinNumber % 8;

        pGPIO_handle->pGPIOx->AFR[temp1] &= (0xF << (4 * temp2));
        pGPIO_handle->pGPIOx->AFR[temp1] |= (pGPIO_handle->GPIO_pinConfig.GPIO_PinAltFunMode << (4 * temp2));
    }
}

void GPIO_deinit(GPIO_regDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
        GPIOG_REG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
        GPIOH_REG_RESET();
    }
}

// GPIO READ/WRITE
uint8_t GPIO_read_from_input_pin(GPIO_regDef_t *pGPIOx, uint8_t pin_number)
{
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> pin_number) & 0x00000001);
    return value;
}
uint16_t GPIO_read_from_input_port(GPIO_regDef_t *pGPIOx)
{
    uint16_t value;
    value = (uint16_t)pGPIOx->IDR;
    return value;
}
void GPIO_write_to_output_pin(GPIO_regDef_t *pGPIOx, uint8_t pin_number, uint8_t value)
{
    if (value == GPIO_PIN_SET)
    {
        //WRITE 1 TO OUTPUT DATA REGISTER AT THE BIT FIELD CORRESPONDING TO THE PIN NUMBER.
        pGPIOx->ODR |= (1 << pin_number);
    }
    else
    {
        //WRITE 0.
        pGPIOx->ODR &= ~(1 << pin_number);
    }
}
void GPIO_write_to_output_port(GPIO_regDef_t *pGPIOx, uint16_t value)
{
    pGPIOx->ODR = value;
}
void GPIO_toggle_output_pin(GPIO_regDef_t *pGPIOx, uint8_t pin_number)
{
    pGPIOx->ODR ^= (1 << pin_number);
}

//// GPIO INTERRUPT HANDLING
void GPIO_IRQ_config(uint8_t IRQ_Number/*, uint8_t irq_priority*/, uint8_t ENorDI)
{
    if (ENorDI == ENABLE)
    {
        if (IRQ_Number <= 31)
        {
            //program ISER0 register.
            *NVIC_ISER0 |= (1 << IRQ_Number);
        }
        else if (IRQ_Number > 31 && IRQ_Number < 64)
        {
            //program ISER1 register.
            *NVIC_ISER1 |= (1 << IRQ_Number % 32);
        }
        else if (IRQ_Number >= 64 && IRQ_Number < 96)
        {
            *NVIC_ISER2 |= (1 << IRQ_Number % 64);
        }
    }
    else
    {
        if (IRQ_Number <= 31)
        {
            *NVIC_ICER0 |= (1 << IRQ_Number);
        }
        else if (IRQ_Number > 31 && IRQ_Number < 64)
        {
            *NVIC_ICER1 |= (1 << IRQ_Number % 32);
        }
        else if (IRQ_Number > 63 && IRQ_Number < 96)
        {
            *NVIC_ICER2 |= (1 << IRQ_Number % 64);
        }
    }
    
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;
    
    uint8_t shift_amount = (8 * iprx_section) + (8 - NUMBER_OF_PRIORITY_BITS_IMPLEMENTED);
    *(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);
    
}
//void GPIO_IRQ_handling(uint8_t pin_number)
//{

//}
