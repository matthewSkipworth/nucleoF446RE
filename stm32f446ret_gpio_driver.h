// stm32f446ret_gpio_driver.h

#ifndef INC_STM32F446RET_GPIO_DRIVER_H
#define INC_STM32F446RET_GPIO_DRIVER_H

#include "stm32f446ret.h"

// configurable items for user application

// GPIO port name
// GPIO pin number
// GPIO mode
// GPIO speed
// GPIO output type
// GPIO Pullup-pulldown
// GPIO Alternate function mode
typedef struct
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;  // @GPIO PIN MODES
    uint8_t GPIO_PinSpeed; // @GPIO OUTPUT SPEEDS
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunMode;

} GPIO_pinConfig_t;

typedef struct
{
    //pointer to hold the base address fo the GPIO peripheral
    GPIO_regDef_t *pGPIOx;           // holds the base address to the gpio port to which the pin belongs
    GPIO_pinConfig_t GPIO_pinConfig; // holds gpio pin configuration settings.

} GPIO_Handle_t;

// @GPIO PIN MODES
#define GPIO_MODE_INPUT 0
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_ALTFUN 2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FE 4
#define GPIO_MODE_IT_RE 5
#define GPIO_MODE_IT_FERE 6

// PUSH-PULL, OPEN-DRAIN SETTINGS
#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD 1

// @GPIO OUTPUT SPEEDS
#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_FAST 2
#define GPIO_SPEED_HIGH 3

// PU/PD RESISTOR SETTINGS
#define GPIO_NO_PUPD 0
#define GPIO_PU 1
#define GPIO_PD 2

// @GPIO PIN NUMBERS

#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

// DRIVER API REQUIREMENTS

// GPIO CLOCK CONTROL
void GPIO_peripheral_clock_control(GPIO_regDef_t *pGPIOx, uint8_t ENorDI);

// GPIO INITIALIZATION
void GPIO_init(GPIO_Handle_t *pGPIO_handle);
void GPIO_deinit(GPIO_regDef_t *pGPIOx);

// GPIO READ/WRITE
uint8_t GPIO_read_from_input_pin(GPIO_regDef_t *pGPIOx, uint8_t pin_number);
uint16_t GPIO_read_from_input_port(GPIO_regDef_t *pGPIOx);
void GPIO_write_to_output_pin(GPIO_regDef_t *pGPIOx, uint8_t pin_number, uint8_t value);
void GPIO_write_to_output_port(GPIO_regDef_t *pGPIOx, uint16_t value);
void GPIO_toggle_output_pin(GPIO_regDef_t *pGPIOx, uint8_t pin_number);

// GPIO INTERRUPT HANDLING
 void GPIO_IRQ_config(uint8_t irq_number/*, uint8_t irq_priority*/, uint8_t ENorDI);
// void GPIO_IRQ_handling(uint8_t pin_number);

// ENABLE/DISABLE GPIO PORT CLOCK
// READ FROM A GPIO PIN
// WRITE TO A GPIO PIN
// CONFIGURE ALTERNATE FUNCTIONALITY
// INTERRUPT HANDLING

#endif
