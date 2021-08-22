#ifndef STM32F44RE_I2C_DRIVER_H
#define STM32F44RE_I2C_DRIVER_H

#include <stdint.h>
#include <stm32f446re.h>

typedef struct 
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


/**
 * I2C_SCL_SPEED 
 */
#define I2C_SCL_SPEED_SM        100000
#define I2C_SCL_SPEED_FM4K      400000
#define I2C_SCL_SPEED_FM2K      200000

/**
 * I2C_ACK_CONTROL
 */
#define I2C_ACK_ENABLE          1
#define I2C_ACK_DISABLE         0

/**
 * I2C_FM_DUTYCYCLE
 */
#define I2C_FM_DUTY_2           0
#define I2C_FM_DUTY_16_9        1


// BIT POSITION MACROS FOR CR1, CR2, DR, SR1, SR2, CCR, TRISE

/**
 * I2C CR1 BIT POSITIONS
 */
#define I2C_CR1_PE_BIT          0
#define I2C_CR1_SMBUS_BIT       1
#define I2C_CR1_SMBTYPE_BIT     3
#define I2C_CR1_ENARP_BIT       4
#define I2C_CR1_ENPEC_BIT       5
#define I2C_CR1_ENGC_BIT        6
#define I2C_CR1_NOSTRETCH_BIT   7
#define I2C_CR1_PE_START_BIT    8
#define I2C_CR1_STOP_BIT        9
#define I2C_CR1_ACK_BIT         10
#define I2C_CR1_POS_BIT         11
#define I2C_CR1_PEC_BIT         12
#define I2C_CR1_ALERT_BIT       13
#define I2C_CR1_SWRST_BIT       15

/**
 * I2C CR2 BIT POSITIONS
 */

#define I2C_CR2_FREQ_BITS        0
#define I2C_CR2_ITRREN_BIT      8
#define I2C_CR2_ITEVTEN_BIT     9
#define I2C_CR2_ITBUFEN_BIT     10
#define I2C_CR2_DMAEN_BIT       11
#define I2C_CR2_LAST_BIT        12

/**
 * I2C SR1 BIT POSITIONS 
 */

#define I2C_SR1_SB_BIT          0
#define I2C_SR1_ADDR_BIT        1
#define I2C_SR1_BTF_BIT         2
#define I2C_SR1_ADD10_BIT       3
#define I2C_SR1_STOPF_BIT       4
#define I2C_SR1_RXNE_BIT        6
#define I2C_SR1_TXE_BIT         7
#define I2C_SR1_BERR_BIT        8
#define I2C_SR1_ARLO_BIT        9
#define I2C_SR1_AF_BIT          10
#define I2C_SR1_OVR_BIT         11
#define I2C_SR1_PECERR_BIT      12
#define I2C_SR1_TIMOUT_BIT      14
#define I2C_SR1_SMBALERT_BIT    15

/**
 * I2C SR2 BIT POSITIONS
 */

#define I2C_SR2_MSL_BIT 0
#define I2C_SR2_BUSY_BIT 1
#define I2C_SR2_TRA_BIT 2
#define I2C_SR2_GENCALL_BIT 4
#define I2C_SR2_SMBDEFAUL_BIT 5
#define I2C_SR2_SMBHOST_BIT 6
#define I2C_SR2_DUALF_BIT 7
#define I2C_SR2_PEC_BITS 8

/**
 * I2C CCR BIT POSITIONS
 */

#define I2C_CCR_CCR_BITS 0
#define I2C_CCR_DUTY_BIT 14
#define I2C_CCR_FS_BIT 15

/**
 * I2C TRISE BIT POSITIONS
 */

#define I2C_TRISE_BITS 0

void I2C_PeriClockControl(I2C_regDef_t* pI2Cx, uint8_t ENorDI);

void I2C_PeripheralControl(I2C_regDef_t* pI2Cx, uint8_t ENorDI);

void I2C_Init(I2C_Handle_t* pHandle);
void I2C_Master_Transmit();
void I2C_Master_Recieve();
void I2C_Slave_Transmit();
void I2C_Slave_Recieve();
void I2C_Error_Interrupt_Handling();
void I2C_Event_Interrupt_Handling();

#endif //STM32F44RE_I2C_DRIVER_H

