#include "stm32f10x.h"
#include "hw_LIS302DL.h"

/* Functions from tilt_sensor.c  */
void GPIOA_Init(void);
void RCC_Init(void);
void Delay(uint32_t Val);
uint8_t Read_Register(uint8_t reg_address);
void Write_Register(uint8_t reg_address, uint8_t data);
void SysTick_Wait(uint32_t delay);
void SysTick_Init(void);
uint8_t Define_Orientation(int8_t x, int8_t y, int8_t z);


#define PRECISION   18  // resolution of measured acceleration in mg
#define G           0x56 // according to LIS302DL range
#define BOTTOM       0
#define TOP          1
#define RIGHT        2
#define LEFT         3


/* Time delays for SysTick Timer  */
#define T1s    8000000
#define T0s5   4000000
#define T0s25  2000000
#define T0s03  240000


/* Selection of Master Receiver mode or
    Master Transmitter mode */
#define WRITE ((uint8_t)0x00)
#define READ  ((uint8_t)0x01)


/* Slave ADdress
   SDO is connected to voltage =>
   LIS302DL ADDRESS 0x0011101h */
#define SAD ((uint8_t)0x3A)
#define SAD_W ((uint8_t)0x3A)
#define SAD_R ((uint8_t)0x3B)

/*========================================
     
                                         FUNCTIONS FROM LCD
                                                        ==========================================*/


#define E                  ((uint8_t) 0x04)
#define RW                 ((uint8_t) 0x02)
#define RS                 ((uint8_t) 0x01)

#define FUNCTION_SET       ((uint8_t) 0x38)     
#define CLEAR_DISPLAY      ((uint8_t) 0x01)
#define ENTRY_MODE_SET     ((uint8_t) 0x06)
#define DISPLAY_ON         ((uint8_t) 0x0C)
#define CURSOR_SHIFT_RIGHT ((uint8_t) 0x14)
#define CURSOR_SHIFT_LEFT  ((uint8_t) 0x10)

#define T500n 4
#define T40u  320
#define T1m53  12300

void LCD_GPIO_Init(void);
void LCD_OutCmd(uint8_t command);
void LCD_OutData(uint8_t letter);                                                         
void LCD_Init(void);
void LCD_OutAccel(uint32_t accel);
void LCD_OutString(char *s);                                                     


/*========================================
     
                                         FUNCTIONS FROM I2C
                                                        ==========================================*/

void I2C1_SoftwareReset(void);
void I2C1_GenerateSTART(void);
void I2C1_GenerateSTOP(void);
void I2C1_SendData(uint8_t Data);
int8_t I2C1_ReceiveData(void);
void I2C1_Init(void);
void I2C1_Send7bitAddress(uint8_t I2C_Direction);
ErrorStatus I2C1_CheckEvent(uint32_t I2C_EVENT);
FlagStatus I2C1_GetFlagStatus(uint32_t I2C_FLAG);
void I2C1_AcknowledgeConfig(FunctionalState NewState);


/*========================================
     
                     I2C Master Events (Events grouped in order of communication)
                                                        ==========================================*/
/** 
  * Communication start
  * 
  * After sending the START condition (I2C1_GenerateSTART() function) the master 
  * has to wait for this event. It means that the Start condition has been correctly 
  * released on the I2C bus (the bus is free, no other devices is communicating).
  * 
  */
/* --EV5 */
#define  I2C_EVENT_MASTER_MODE_SELECT                      ((uint32_t)0x00030001)  /* BUSY, MSL and SB flag */

/** 
  * Address Acknowledge
  * 
  * After checking on EV5 (start condition correctly released on the bus), the 
  * master sends the address of the slave(s) with which it will communicate 
  * (I2C_Send7bitAddress() function, it also determines the direction of the communication: 
  * Master transmitter or Receiver). Then the master has to wait that a slave acknowledges 
  * his address. If an acknowledge is sent on the bus, one of the following events will 
  * be set:
  * 
  *  1) In case of Master Receiver (7-bit addressing): the I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED 
  *     event is set.
  *  
  *  2) In case of Master Transmitter (7-bit addressing): the I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED 
  *     is set
  *     
  */

/* --EV6 */
#define  I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED        ((uint32_t)0x00070082)  /* BUSY, MSL, ADDR, TXE and TRA flags */
#define  I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED           ((uint32_t)0x00030002)  /* BUSY, MSL and ADDR flags */


/** 
  * Communication events
  * 
  * If a communication is established (START condition generated and slave address 
  * acknowledged) then the master has to check on one of the following events for 
  * communication procedures:
  *  
  * 1) Master Receiver mode: The master has to wait on the event EV7 then to read 
  *    the data received from the slave (I2C_ReceiveData() function).
  * 
  * 2) Master Transmitter mode: The master has to send data (I2C_SendData() 
  *    function) then to wait on event EV8 or EV8_2.
  *    These two events are similar: 
  *     - EV8 means that the data has been written in the data register and is 
  *       being shifted out.
  *     - EV8_2 means that the data has been physically shifted out and output 
  *       on the bus.
  *     In most cases, using EV8 is sufficient for the application.
  *     Using EV8_2 leads to a slower communication but ensure more reliable test.
  *     EV8_2 is also more suitable than EV8 for testing on the last data transmission 
  *     (before Stop condition generation).
  *     
  *  Note: In case the  user software does not guarantee that this event EV7 is 
  *  managed before the current byte end of transfer, then user may check on EV7 
  *  and BTF flag at the same time (ie. (I2C_EVENT_MASTER_BYTE_RECEIVED | I2C_FLAG_BTF)).
  *  In this case the communication may be slower.
  * 
  */

/* Master RECEIVER mode -----------------------------*/ 
/* --EV7 */
#define  I2C_EVENT_MASTER_BYTE_RECEIVED                    ((uint32_t)0x00030040)  /* BUSY, MSL and RXNE flags */

/* Master TRANSMITTER mode --------------------------*/
/* --EV8 */
#define I2C_EVENT_MASTER_BYTE_TRANSMITTING                 ((uint32_t)0x00070080) /* TRA, BUSY, MSL, TXE flags */
/* --EV8_2 */
#define  I2C_EVENT_MASTER_BYTE_TRANSMITTED                 ((uint32_t)0x00070084)  /* TRA, BUSY, MSL, TXE and BTF flags */


/** I2C_flags_definition 
  * 
  */

/** 
  *  brief  SR2 register flags  
  */

#define I2C_FLAG_DUALF                  ((uint32_t)0x00800000)
#define I2C_FLAG_SMBHOST                ((uint32_t)0x00400000)
#define I2C_FLAG_SMBDEFAULT             ((uint32_t)0x00200000)
#define I2C_FLAG_GENCALL                ((uint32_t)0x00100000)
#define I2C_FLAG_TRA                    ((uint32_t)0x00040000)
#define I2C_FLAG_BUSY                   ((uint32_t)0x00020000)
#define I2C_FLAG_MSL                    ((uint32_t)0x00010000)

/** 
  *  brief  SR1 register flags  
  */

#define I2C_FLAG_SMBALERT               ((uint32_t)0x10008000)
#define I2C_FLAG_TIMEOUT                ((uint32_t)0x10004000)
#define I2C_FLAG_PECERR                 ((uint32_t)0x10001000)
#define I2C_FLAG_OVR                    ((uint32_t)0x10000800)
#define I2C_FLAG_AF                     ((uint32_t)0x10000400)
#define I2C_FLAG_ARLO                   ((uint32_t)0x10000200)
#define I2C_FLAG_BERR                   ((uint32_t)0x10000100)
#define I2C_FLAG_TXE                    ((uint32_t)0x10000080)
#define I2C_FLAG_RXNE                   ((uint32_t)0x10000040)
#define I2C_FLAG_STOPF                  ((uint32_t)0x10000010)
#define I2C_FLAG_ADD10                  ((uint32_t)0x10000008)
#define I2C_FLAG_BTF                    ((uint32_t)0x10000004)
#define I2C_FLAG_ADDR                   ((uint32_t)0x10000002)
#define I2C_FLAG_SB                     ((uint32_t)0x10000001)

/*
 * I2C1 Status Registers Addresses
 */

#define I2C1_SR1                        (I2C1_BASE+0x14)
#define I2C1_SR2                        (I2C1_BASE+0x18)



