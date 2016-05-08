/**
  ******************************************************************************
  * file:     i2c_driver.c
  * author:   Mikhail Mishin
  * version:  V1.0
  * date:     30-May-2015
  * brief:    This file provides the I2C firmware functions for I2C1 Master Mode.
  ******************************************************************************
                                                                              */

/* Includes ------------------------------------------------------------------*/

#include "tilt_sensor.h"

/* I2C FLAG mask */
#define FLAG_Mask               ((uint32_t)0x00FFFFFF)


void I2C1_GPIO_Init(void);


// SET_BIT(REG, BIT)     ((REG) |= (BIT))
// CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
// READ_BIT(REG, BIT)    ((REG) & (BIT))
// CLEAR_REG(REG)        ((REG) = (0x0))
// WRITE_REG(REG, VAL)   ((REG) = (VAL))
// READ_REG(REG)         ((REG))
// MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))


/* Initializes the I2C1 peripheral according to the specified parameters in master mode */
void I2C1_Init(void)
{
    // PB6 and PB7 configuration
    I2C1_GPIO_Init();
    // I2C 1 clock enable
	SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);

/*---------------------------- I2C1 CR2 Configuration -----------------------------------------*
* Program the peripheral input clock in I2C1_CR2 Register in order to generate correct timings *
  
  FREQ[5:0]: Peripheral clock frequency
	The peripheral clock frequency must be configured using the input APB clock frequency (I2C
	peripheral connected to APB). The minimum allowed frequency is 2 MHz, the maximum
	frequency is limited by the maximum APB frequency (24 MHz) and an intrinsic limitation of
	46 MHz.
  I2C_CR2_FREQ_3   ((uint16_t)0x0008): 8 MHz */ 
  MODIFY_REG(I2C1->CR2, I2C_CR2_FREQ, I2C_CR2_FREQ_3);


/*---------------------------- I2C1 CCR Configuration -----------------------------------------*       
* Configure the clock control registers
  Note: 1 f_PCLK1 must be at least 2 MHz to achieve standard mode I²C frequencies. It must be at
          least 4 MHz to achieve fast mode I²C frequencies. It must be a multiple of 10MHz to reach 
          the 400 kHz maximum I²C fast mode clock.
        2 The CCR register must be configured only when the I2C is disabled (PE = 0).
        
  Disable the selected I2C peripheral to configure TRISE 
  I2C_CR1_PE   ((uint16_t)0x0001)  Peripheral Enable                                             */
  CLEAR_BIT(I2C1->CR1, I2C_CR1_PE);
  
  /* Clear F/S, DUTY and CCR[11:0] bits */
  CLEAR_BIT(I2C1->CCR, I2C_CCR_FS | I2C_CCR_DUTY | I2C_CCR_CCR);

  /* Configure speed in standard mode 
  CCR[11:0]: Clock control register in Fast/Standard mode (Master mode)
             Controls the SCL clock in master mode.

             Standard mode or SMBus:
             T_high = CCR * TPCLK1
             T_low = CCR * TPCLK1
  For instance: in standard mode, to generate a 100 kHz SCL frequency:
  If FREQR = 08, TPCLK1 = 125 ns so CCR must be programmed with 0x28
  (0x28 <=> 40d x 125 ns = 5000 ns.)
  Note: 1. The minimum allowed value in standart mode is 0x04 */
  
  SET_BIT(I2C1->CCR, (uint16_t) 0x28);
  
  /*---------------------------- I2C1 TRISE Configuration -----------------------------------------*
  Configure the rise time register
  TRISE[5:0]: Maximum rise time in Fast/Standard mode (Master mode)
        These bits must be programmed with the maximum SCL rise time given in the I2C bus
        specification, incremented by 1.
  For instance: in standard mode, the maximum allowed SCL rise time is 1000 ns.
  If, in the I2C_CR2 register, the value of FREQ[5:0] bits is equal to 0x08 and TPCLK1 = 125 ns
  therefore the TRISE[5:0] bits must be programmed with 09h. (1000 ns / 125 ns = 8 + 1)
  The filter value can also be added to TRISE[5:0].
  If the result is not an integer, TRISE[5:0] must be programmed with the integer part, in order
  to respect the tHIGH parameter.*/
  MODIFY_REG(I2C1->TRISE, I2C_TRISE_TRISE, (uint8_t) 0x09);



/*---------------------------- I2C1 CR1 Configuration ------------------------*/

  CLEAR_BIT(I2C1->CR1, I2C_CR1_SMBUS | I2C_CR1_SMBTYPE);

    /* Program the I2C_CR1 register to enable the peripheral */
  SET_BIT(I2C1->CR1, I2C_CR1_PE);
}

/**
  * brief  Enables or disables the specified I2C acknowledge feature.
  * param  NewState: new state of the I2C Acknowledgement.
  *   This parameter can be: ENABLE or DISABLE.
  * return None.
  */
void I2C1_AcknowledgeConfig(FunctionalState NewState)
{
  if (NewState != DISABLE)
  {
    /* Enable the acknowledgement */
    SET_BIT(I2C1->CR1, I2C_CR1_ACK);
  }
  else
  {
    /* Disable the acknowledgement */
    CLEAR_BIT(I2C1->CR1, I2C_CR1_ACK);
  }
}

/**
  * brief  Software reset.
  * return None.
  */
void I2C1_SoftwareReset()
{
    int i = 100;
    SET_BIT(I2C1->CR1, I2C_CR1_SWRST);
    for (; i !=0; i--){
        __nop();
    }
    CLEAR_BIT(I2C1->CR1, I2C_CR1_SWRST);
}



void I2C1_GPIO_Init(void){
  /*I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA  
    GPIO Configuration: Alternative function open drain */
    
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);

    // GPIOB CRL Configuration
    // 11: Alternate function output Open-drain
    SET_BIT(GPIOB->CRL, GPIO_CRL_CNF6);
    SET_BIT(GPIOB->CRL, GPIO_CRL_CNF7);
    
    // 10: Output mode, max speed 2 MHz.
    //CLEAR_BIT(GPIOB->CRL, GPIO_CRL_MODE6);
    //CLEAR_BIT(GPIOB->CRL, GPIO_CRL_MODE7);
    //SET_BIT(GPIOB->CRL, GPIO_CRL_MODE6_1);
    //SET_BIT(GPIOB->CRL, GPIO_CRL_MODE7_1);
    
    // 11: Output mode, max speed 50 MHz.
    SET_BIT(GPIOB->CRL, GPIO_CRL_MODE6);
    SET_BIT(GPIOB->CRL, GPIO_CRL_MODE7);
}


/* Generates I2C1 communication START condition. */
void I2C1_GenerateSTART(void)
{

    // Generate a START condition 
    SET_BIT(I2C1->CR1, I2C_CR1_START);
}


/* Generates I2C1 communication STOP condition. */
void I2C1_GenerateSTOP(void)
{
    // Generate a STOP condition 
    SET_BIT(I2C1->CR1, I2C_CR1_STOP);
}


/* Transmits the address byte to select the slave device.
*  Arguments:
  *          I2C_Direction: specifies whether the I2C device will be a
  *   Transmitter or a Receiver. (Read or Write)
  */
void I2C1_Send7bitAddress( uint8_t I2C_Direction)
{
  /* Test on the direction to set/reset the read/write bit */
  if (I2C_Direction != WRITE)
  {
    /* Set the address bit0 for read */
    WRITE_REG(I2C1->DR, SAD_R); 
  }
  else
  {
    /* Reset the address bit0 for write */
    WRITE_REG(I2C1->DR, SAD_W);
  }
}


/**
  * brief  Sends a data byte through the I2C1 peripheral.
  * param  Data: Byte to be transmitted..
  * returm None
  */
void I2C1_SendData(uint8_t Data)
{

  /* Write in the DR register the data to be sent */
  WRITE_REG(I2C1->DR, Data);
}

/**
  * brief  Returns the most recent received data by the I2C1 peripheral.
  * return The value of the received data.
  */
int8_t I2C1_ReceiveData(void)
{
  /* Return the data in the DR register */
  return (int8_t) READ_REG(I2C1->DR);
}


/*
 ****************************************************************************************
 *
 *                         I2C State Monitoring Functions
 *                       
 ****************************************************************************************   
 * This I2C driver provides two different ways for I2C state monitoring
 *  depending on the application requirements and constraints:
 *        
 *  
 * 1) Basic state monitoring:
 *    Using I2C_CheckEvent() function:
 *    It compares the status registers (SR1 and SR2) content to a given event
 *    (can be the combination of one or more flags).
 *    It returns SUCCESS if the current status includes the given flags 
 *    and returns ERROR if one or more flags are missing in the current status.
 *    - When to use:
 *      - This function is suitable for most applications as well as for startup 
 *      activity since the events are fully described in the product reference manual 
 *      (RM0008).
 *      - It is also suitable for users who need to define their own events.
 *    - Limitations:
 *      - If an error occurs (ie. error flags are set besides to the monitored flags),
 *        the I2C_CheckEvent() function may return SUCCESS despite the communication
 *        hold or corrupted real state. 
 *
 * 2) Flag-based state monitoring:
 *     Using the function I2C_GetFlagStatus() which simply returns the status of 
 *     one single flag (ie. I2C_FLAG_RXNE ...). 
 *     - When to use:
 *        - This function could be used for specific applications or in debug phase.
 *        - It is suitable when only one flag checking is needed (most I2C events 
 *          are monitored through multiple flags).
 *     - Limitations: 
 *        - When calling this function, the Status register is accessed. Some flags are
 *          cleared when the status register is accessed. So checking the status
 *          of one Flag, may clear other ones.
 *        - Function may need to be called twice or more in order to monitor one 
 *          single event.
 *
 *  For detailed description of Events, please refer to section I2C_Events in 
 *  tilt_sensor.h file.
 *  
 */

/**
 * 
 *  1) Basic state monitoring
 *******************************************************************************
 */

/**
  * Checks whether the last I2C1 Event is equal to the one passed
  *   as parameter.
  * I2C_EVENT: specifies the event to be checked. 
  * This parameter can be one of the following values:
  *     I2C_EVENT_MASTER_MODE_SELECT                          : EV5
  *     I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED            : EV6     
  *     I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED               : EV6
  *     I2C_EVENT_MASTER_BYTE_RECEIVED                        : EV7
  *     I2C_EVENT_MASTER_BYTE_TRANSMITTING                    : EV8
  *     I2C_EVENT_MASTER_BYTE_TRANSMITTED                     : EV8_2
  *
  * An ErrorStatus enumeration value:
  * - SUCCESS: Last event is equal to the I2C_EVENT
  * - ERROR: Last event is different from the I2C_EVENT
  */

ErrorStatus I2C1_CheckEvent(uint32_t I2C_EVENT)
{
  uint32_t last_event = 0;
  ErrorStatus status = ERROR;

  /* Read the I2C1 status register and get the last event value*/
  last_event = (READ_REG(I2C1->SR1) | (READ_REG(I2C1->SR2) << 16)) & FLAG_Mask;

  /* Check whether the last event contains the I2C_EVENT */
  if ((last_event & I2C_EVENT) == I2C_EVENT)
  {
    /* SUCCESS: last event is equal to I2C_EVENT */
    status = SUCCESS;
  }
  else
  {
    /* ERROR: last event is different from I2C_EVENT */
    status = ERROR;
  }
  /* Return status */
  return status;
}


/**
 * 
 *  2) Flag-based state monitoring
 *******************************************************************************
 */

/**
  * brief:  Checks whether the specified I2C flag is set or not.
  * param:  I2C_FLAG: specifies the flag to check. 
  *   This parameter can be one of the following values:
  *      I2C_FLAG_TRA: Transmitter/Receiver flag
  *      I2C_FLAG_BUSY: Bus busy flag
  *      I2C_FLAG_MSL: Master/Slave flag
  *      I2C_FLAG_SMBALERT: SMBus Alert flag
  *      I2C_FLAG_TIMEOUT: Timeout or Tlow error flag
  *      I2C_FLAG_PECERR: PEC error in reception flag
  *      I2C_FLAG_AF: Acknowledge failure flag
  *      I2C_FLAG_ARLO: Arbitration lost flag (Master mode)
  *      I2C_FLAG_BERR: Bus error flag
  *      I2C_FLAG_TXE: Data register empty flag (Transmitter)
  *      I2C_FLAG_RXNE: Data register not empty (Receiver) flag
  *      I2C_FLAG_BTF: Byte transfer finished flag
  *      I2C_FLAG_ADDR: Address sent flag (Master mode) "ADSL"
  *      I2C_FLAG_SB: Start bit flag (Master mode)
  * return: The new state of I2C_FLAG (SET or RESET).
  */
  
FlagStatus I2C1_GetFlagStatus(uint32_t I2C_FLAG)
{
  FlagStatus bitstatus = RESET;
  /* sr_number - number of the I2C1 Status Register (1 - SR1; 0 - SR2)
     sr_base -  base address of the I2C1 Status Register */ 
  __IO uint32_t sr_number = 0 , sr_base=0;   
  
  /* Read flag register index */
  sr_number = I2C_FLAG >> 28;
  
  /* Get bit[23:0] of the flag */
  I2C_FLAG &= READ_BIT(I2C_FLAG, FLAG_Mask);
  
  if(sr_number != 0)
  {
    /* Get the I2Cx SR1 register address */
    sr_base = I2C1_SR1;
  }
  else
  {
    /* Flag in I2Cx SR2 Register */
    I2C_FLAG = (uint32_t)(I2C_FLAG >> 16);
    /* Get the I2Cx SR2 register address */
    sr_base = I2C1_SR2;
  }
  
  if( (READ_REG(*(__IO uint32_t *)sr_base) & I2C_FLAG) != (uint32_t)RESET)
  {
    /* I2C_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* I2C_FLAG is reset */
    bitstatus = RESET;
  }
  
  /* Return the I2C_FLAG status */
  return  bitstatus;
}


