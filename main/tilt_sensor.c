/**
  ******************************************************************************
  * file:     tilt_sensor.c
  * author:   Mikhail Mishin
  * version:  V1.0
  * date:     30-May-2015
  * brief:    This file provides function for running the tilt_sensor device.
  ******************************************************************************
                                                                              */

/* Includes ------------------------------------------------------------------*/

#include "tilt_sensor.h"
#include <stdlib.h>


void tilt_sensor(void)
{
    //uint8_t who_am_i = 0;
    uint8_t  counter, state;
    __IO int8_t x = 0, y = 0, z = 0;
    RCC_Init();
    SysTick_Init();
    GPIOA_Init();
    SysTick_Wait(T1s); // 1 s
    WRITE_REG(GPIOA->BSRR, GPIO_BSRR_BS3);		 
    SysTick_Wait(T1s); // 1 s
    WRITE_REG(GPIOA->BSRR, GPIO_BSRR_BR3);	
    LCD_Init();
    I2C1_Init();
    //who_am_i = Read_Register(WHO_AM_I);
    //Read_Register(CTRL_REG2);
    Write_Register(CTRL_REG1, 0x47);
    SysTick_Wait(T0s03); // Turn-on time 3/ODR = 0.03 s 
    //Read_Register(STATUS_REG);
    //x = Read_Register(OUT_X);
    //y = Read_Register(OUT_Y);
    //z = Read_Register(OUT_Z); 
    while(1)  {
               x = Read_Register(OUT_X);
               y = Read_Register(OUT_Y);
               z = Read_Register(OUT_Z);
               state = Define_Orientation(x, y, z);
               LCD_OutCmd(CLEAR_DISPLAY);
               SysTick_Wait(T1m53);   
               LCD_OutCmd(CURSOR_SHIFT_RIGHT);
               SysTick_Wait(T40u);
                switch (state){
                    case BOTTOM:
                        LCD_OutString("Bottom");
                        break;
                    case TOP:
                        LCD_OutString("Top");
                        break;
                    case RIGHT:
                        LCD_OutString("Right");
                        break;
                    case LEFT:
                        LCD_OutString("Left");
                        break;            
                    default:
                        LCD_OutString("Bottom");
                }
                SysTick_Wait(T0s25); 
                WRITE_REG(GPIOA->BSRR, GPIO_BSRR_BS3);		 
                SysTick_Wait(T0s25); 
                WRITE_REG(GPIOA->BSRR, GPIO_BSRR_BR3);
                
    }
    
    
   /* while(1)  {
               x = Read_Register(OUT_X);
               y = Read_Register(OUT_Y);
               z = Read_Register(OUT_Z);
               LCD_OutAccel((uint32_t) abs(z)*PRECISION);
               SysTick_Wait(T0s5);
                for (counter = 6; counter != 0; counter--)
                {
                    LCD_OutCmd(CURSOR_SHIFT_LEFT);
                    SysTick_Wait(T40u);
                }
    }*/
    /*LCD_Init();
    LCD_Write1();
    while(1)  {
            WRITE_REG(GPIOA->BSRR, GPIO_BSRR_BS3);		// set bit
            SysTick_Wait(T0s5); // 0.5 s
            WRITE_REG(GPIOA->BSRR, GPIO_BSRR_BR3);		// reset bit 
            SysTick_Wait(T0s5); // 0.5 s
	}    */ 
}

void GPIOA_Init(void)
{
    // // Port A clock enable
	SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);

    // GPIOA CRL Configuration
    // 00: General purpose output push-pull
    CLEAR_BIT(GPIOA->CRL, GPIO_CRL_CNF3);
	// 01: Output mode, max speed 10 MHz.
    CLEAR_BIT(GPIOA->CRL, GPIO_CRL_MODE3);
	SET_BIT(GPIOA->CRL, GPIO_CRL_MODE3_0);
}

void RCC_Init(void) 
{
    /* Reset the RCC clock configuration to the default reset state(for debug purpose) */
    /* Set HSION bit */
    
    SET_BIT(RCC->CR, RCC_CR_HSION);

    /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
    CLEAR_BIT(RCC->CFGR, RCC_CFGR_SW | RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 | RCC_CFGR_ADCPRE | RCC_CFGR_MCO);
  
    /* Reset HSEON, CSSON and PLLON bits */
    CLEAR_BIT(RCC->CR, RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON); 

    /* Reset HSEBYP bit */
    CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);

    /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
    CLEAR_BIT(RCC->CFGR, RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);  
    //RCC->CFGR &= (uint32_t)0xFF80FFFF;
    
    /* Disable all interrupts and clear pending bits  */
    RCC->CIR = 0x009F0000;

    /* Reset CFGR2 register */
    RCC->CFGR2 = 0x00000000;  
    
    /* Prefetch Buffer Enable */
    SET_BIT(FLASH->ACR, FLASH_ACR_PRFTBE);
    
    MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2);
    
    /* HCLK = SYSCLK 															*/
    /* AHB Prescaler = 1         */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_DIV1);	

    /* PCLK2 = HCLK 															*/
    /* APB2 Prescaler = 1        */
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_DIV1);	
  
    /* PCLK1 = HCLK 															*/
    /* APB1 Prescaler = 1 														*/
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV1);		
}


uint8_t Read_Register(uint8_t reg_address){
    
    int8_t received_data = 0;
  /* Re-enable ACK bit incase it was disabled last call*/
    I2C1_AcknowledgeConfig(ENABLE);
  /* Check flag BUSY = 0*/
    while (I2C1_GetFlagStatus(I2C_FLAG_BUSY));
  /* Send START (ST) condition */ 
    I2C1_GenerateSTART();
  /* EV5 SB=1, cleared by reading SR1 register followed by writing DR register with Address*/
    while( ! I2C1_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
  /* Send SAD+W */
    I2C1_Send7bitAddress(WRITE);
  /* EV6, ADDR=1, cleared by reading SR1 register followed by reading SR2 register. */
    while(!I2C1_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  /* Send the device's regiser address SUB to write to */
    I2C1_SendData(reg_address);
  /* EV8 TxE=1, shift register not empty, data register empty, cleared by writing DR register */
    while( ! I2C1_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING));
/*=====================================================*/
  /* Send START condition a second time (Re-Start) */
    I2C1_GenerateSTART();
  /* EV5 SB=1, cleared by reading SR1 register followed by writing DR register with Address*/
    while( ! I2C1_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
  /* Send SUB+R */
    I2C1_Send7bitAddress(READ);
  /* EV6 ADDR=1, cleared by reading SR1 register followed by reading SR2*/
    while( !I2C1_CheckEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  /* EV6_1 No associated flag event, used for 1 byte reception only. The Acknowledge disable and Stop 
     condition generation are made just after EV6, that is after ADDR is cleared*/
     I2C1_AcknowledgeConfig(DISABLE);
  /* Send STOP Condition */
     I2C1_GenerateSTOP();
  
  /*EV7 RxNE=1 cleared by reading DR register */
    while (!I2C1_GetFlagStatus(I2C_FLAG_RXNE));
    received_data = I2C1_ReceiveData();  
  
    return received_data;
}


void Write_Register(uint8_t reg_address, uint8_t data){

  /* Re-enable ACK bit incase it was disabled last call*/
    I2C1_AcknowledgeConfig(ENABLE);
  /* Check flag BUSY = 0*/
    while (I2C1_GetFlagStatus(I2C_FLAG_BUSY));
  /* Send START (ST) condition */ 
    I2C1_GenerateSTART();
  /* EV5 SB=1, cleared by reading SR1 register followed by writing DR register with Address*/
    while( ! I2C1_CheckEvent(I2C_EVENT_MASTER_MODE_SELECT));
  /* Send SAD+W */
    I2C1_Send7bitAddress(WRITE);
  /* EV6, ADDR=1, cleared by reading SR1 register followed by reading SR2 register. */
    while(!I2C1_CheckEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  /* Send the device's regiser address SUB to write to */
    I2C1_SendData(reg_address);
  /* EV8 TxE=1, shift register not empty, data register empty, cleared by writing DR register */
    while( ! I2C1_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTING));
  /* Send the DATA */
    I2C1_SendData(data);      
  /* EV8_2 TxE=1, BTF=1 Program Srop request. TxE and BTF are cleared by hardware by the Stop condition */
    while( ! I2C1_CheckEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED));  
  /* Generate Stop Condition */  
    I2C1_GenerateSTOP();
  
}


void Delay( uint32_t Val) 
{
    for( ; Val != 0; Val--) {
		    __nop();
  	}
}


/**
 * brief  Initialize and start the SysTick counter and its interrupt.
 *
 * param   ticks   number of ticks between two interrupts
 *
 * Initialise the system tick timer and its interrupt and start the
 * system tick timer / counter in free running mode not generating 
 * interrupts.
 */
void SysTick_Init(void)
{ 
    /* Disable SysTick during setup */
  CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE);      
    /* Set reload register ((uint32_t)0x00FFFFFF) */
  WRITE_REG(SysTick->LOAD, SysTick_LOAD_RELOAD);    
    /* Any write to this register clears it */
  WRITE_REG(SysTick->VAL, 0); 
    /* Disable exception request and select Processor Clock (AHB) source clock */ 
  SET_BIT(SysTick->CTRL, SysTick_CTRL_CLKSOURCE);
  CLEAR_BIT(SysTick->CTRL, SysTick_CTRL_TICKINT); 
    /* Enable SysTick Timer */   
  SET_BIT(SysTick->CTRL, SysTick_CTRL_ENABLE);                                                   
}


/**
 * brief  Generates time delay
 *
 * param   delay   the delay parameter in units if the 8MHz clock (125ns)
 *                 delay must be less than 16777216!!
 */
void SysTick_Wait(uint32_t delay){
    volatile uint32_t elapsed_time;
    uint32_t start_time = READ_REG(SysTick->VAL);
    do{
        elapsed_time = (start_time - READ_REG(SysTick->VAL))&0x00FFFFFF;
    }
    while (elapsed_time <= delay);
}


/**
 * brief  A simple state machine in which the accelerometer reading
           leads to transitions between the screen orientation.
 *
 * param  x, y, z - accelerometer readings
* return state : 0 - Bottom
                 1 - Top
                 2 - Right
                 3 - Left
 */

uint8_t Define_Orientation(int8_t x, int8_t y, int8_t z)
{
    uint8_t state = BOTTOM; // The default state is Bottom
    x *= -1;
    if ( (abs(z) < 0.5*G) && (x > 0.5*G) && (abs(y) < 0.4*G) ) 
        {
            state = TOP; // Change orientation to Top
        }
    else if ( (abs(z) < 0.5*G) && (y > 0.5*G) && (abs(x) < 0.4*G) ) 
        {
            state = RIGHT; // Change orientation to Right
        }
    else if ( (abs(z) < 0.5*G) && (y < -0.5*G) && (abs(x) < 0.4*G) ) 
        {
            state = LEFT; // Change orientation to Left
        }
    return state;
    }
        







