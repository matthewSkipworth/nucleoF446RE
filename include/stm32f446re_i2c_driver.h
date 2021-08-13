#ifndef STM32F44RE_I2C_DRIVER_H
#define STM32F44RE_I2C_DRIVER_H


typedef struct 
{
    uint8_t SCL_Speed;
    uint8_t DeviceAddress;
    uint8_t ACK_Control;
    uint8_t FM_DutyCycle;
} I2C_Config_t;


// BIT POSITION MACROS FOR CR1, CR2, DR, SR1, SR2, CCR, TRISE

void I2C_Init();
void I2C_Master_Transmit();
void I2C_Master_Recieve();
void I2C_Slave_Transmit();
void I2C_Slave_Recieve();
void I2C_Error_Interrupt_Handling();
void I2C_Event_Interrupt_Handling();

#endif //STM32F44RE_I2C_DRIVER_H

