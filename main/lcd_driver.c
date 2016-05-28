/**
  ******************************************************************************
  * file:     lcd_driver.c
  * author:   Mikhail Mishin
  * version:  V1.0
  * date:     30-May-2015
  * brief:    This file provides functions for working with LCD display.
  ******************************************************************************
                                                                              */

/* Includes ------------------------------------------------------------------*/

#include "tilt_sensor.h"

void __out_data(uint8_t data);
void __out_ctrl(uint8_t data);

void LCD_Init(void){
    LCD_GPIO_Init();
    SysTick_Wait(T40u);
    LCD_OutCmd(FUNCTION_SET); // Set interface data length
    SysTick_Wait(T40u);
    LCD_OutCmd(CLEAR_DISPLAY); // Clear display
    SysTick_Wait(T1m53);
    LCD_OutCmd(ENTRY_MODE_SET); // Assign cursor moving direction right and disable the
                                // shift of the entire display
    SysTick_Wait(T40u);    
    LCD_OutCmd(DISPLAY_ON); // Set display control bit
    SysTick_Wait(T40u);
    LCD_OutCmd(CLEAR_DISPLAY); // Clear display
    SysTick_Wait(T1m53);
}


/**
 * brief  Write decimal to the LCD
 *
 * param   d - decimal number from 0 to 9
 */
void LCD_OutDec(d){
    LCD_OutData(d + 0x30);
}

/**
 * brief  Write string to the LCD
 *
 * param   s  - string
 */
void LCD_OutString(char *s){
     for (; *s != '\0'; s++){
         LCD_OutData(*s);
     }
}


/**
 * brief  Write character to the LCD
 *
 * param   c - character
 */
void LCD_OutAccel(uint32_t accel){
  
    LCD_OutDec(accel/1000);
    SysTick_Wait(T40u);
    LCD_OutData('.');
    SysTick_Wait(T40u);
    LCD_OutDec((accel%1000)/100);
    SysTick_Wait(T40u);
    LCD_OutDec((accel%100)/10);
    SysTick_Wait(T40u);
    LCD_OutDec(accel%10);
    SysTick_Wait(T40u);
    LCD_OutData('g');
    SysTick_Wait(T40u);
}



/* send a command byte to the LCD */
void LCD_OutCmd(uint8_t command){
    
    __out_ctrl(0);
    SysTick_Wait(T500n);
    __out_ctrl(E);
    __out_data(command);
    SysTick_Wait(T500n);
    __out_ctrl(0);
    SysTick_Wait(T40u);
}


/* send a command byte to the LCD */
void LCD_OutData(uint8_t letter){
    
    __out_ctrl(RS);          // E=0, RW=0, RS =1
    SysTick_Wait(T500n);
    __out_ctrl(E+RS);        // E=1, RW=0, RS =1
    __out_data(letter);
    SysTick_Wait(T500n);
    __out_ctrl(RS);          // E=0, RW=0, RS =1
    SysTick_Wait(T40u);
}


/* write data to the data lines
   DB7   DB6    DB5    DB4    DB3    DB2   DB1   DB0
   PA12  PA11   PA10   PA9    PA8    PB15  PB14  PB13 */
void __out_data(uint8_t data){
    uint32_t gpiob_bsrr_mask = 0x00000000;
    uint32_t gpioa_bsrr_mask = 0x00000000;  
    /* PB13 - DB0 */
    if (data & 0x01) {
        SET_BIT(gpiob_bsrr_mask, GPIO_BSRR_BS13);    // set bit
    }
    else{
        SET_BIT(gpiob_bsrr_mask, GPIO_BSRR_BR13);    // reset bit
    }
    data >>= 1;
    /* PB14 - DB1 */
    if (data & 0x01) {
        SET_BIT(gpiob_bsrr_mask, GPIO_BSRR_BS14);    // set bit
    }
    else{
        SET_BIT(gpiob_bsrr_mask, GPIO_BSRR_BR14);    // reset bit
    }
    data >>= 1;
    /* PB15 - DB2 */
    if (data & 0x01) {
        SET_BIT(gpiob_bsrr_mask, GPIO_BSRR_BS15);    // set bit
    }
    else{
        SET_BIT(gpiob_bsrr_mask, GPIO_BSRR_BR15);    // reset bit
    }
    data >>= 1;
    /* PA8 - DB3 */
    if (data & 0x01) {
        SET_BIT(gpioa_bsrr_mask, GPIO_BSRR_BS8);    // set bit
    }
    else{
        SET_BIT(gpioa_bsrr_mask, GPIO_BSRR_BR8);    // reset bit
    }
    data >>= 1;
    /* PA9 - DB4 */
    if (data & 0x01) {
        SET_BIT(gpioa_bsrr_mask, GPIO_BSRR_BS9);    // set bit
    }
    else{
        SET_BIT(gpioa_bsrr_mask, GPIO_BSRR_BR9);    // reset bit
    }
    data >>= 1;
    /* PA10 - DB5 */
    if (data & 0x01) {
        SET_BIT(gpioa_bsrr_mask, GPIO_BSRR_BS10);    // set bit
    }
    else{
        SET_BIT(gpioa_bsrr_mask, GPIO_BSRR_BR10);    // reset bit
    }
    data >>= 1;
    /* PA11 - DB6 */
    if (data & 0x01) {
        SET_BIT(gpioa_bsrr_mask, GPIO_BSRR_BS11);    // set bit
    }
    else{
        SET_BIT(gpioa_bsrr_mask, GPIO_BSRR_BR11);    // reset bit
    }
    data >>= 1;
    /* PA12 - DB7 */
    if (data & 0x01) {
        SET_BIT(gpioa_bsrr_mask, GPIO_BSRR_BS12);    // set bit
    }
    else{
        SET_BIT(gpioa_bsrr_mask, GPIO_BSRR_BR12);    // reset bit
    }
    WRITE_REG(GPIOA->BSRR, gpioa_bsrr_mask);
    WRITE_REG(GPIOB->BSRR, gpiob_bsrr_mask);
    
}
 

/* write data to the control lines
   E     RW     RS    
   PB12  PB11   PB10  */
void __out_ctrl(uint8_t data){
    /* PB10 - RS */
    uint32_t gpiob_bsrr_mask = 0x00000000;
    if (data & 0x01) {
        SET_BIT(gpiob_bsrr_mask, GPIO_BSRR_BS10);    // set bit
    }
    else{
        SET_BIT(gpiob_bsrr_mask, GPIO_BSRR_BR10);    // reset bit
    }
    data >>= 1;
    /* PB11 - RW */
    if (data & 0x01) {
        SET_BIT(gpiob_bsrr_mask, GPIO_BSRR_BS11);    // set bit
    }
    else{
        SET_BIT(gpiob_bsrr_mask, GPIO_BSRR_BR11);    // reset bit
    }
    data >>= 1;
    /* PB12 - E */
    if (data & 0x01) {
        SET_BIT(gpiob_bsrr_mask, GPIO_BSRR_BS12);    // set bit
    }
    else{
        SET_BIT(gpiob_bsrr_mask, GPIO_BSRR_BR12);    // reset bit
    }
    WRITE_REG(GPIOB->BSRR, gpiob_bsrr_mask);
}


void LCD_GPIO_Init(void){
  /*LCD GPIO Configuration    
    PA12     ------> DB7
    PA11     ------> DB6 
    PA10     ------> DB5
    PA9      ------> DB4
    PA8      ------> DB3
    PB15     ------> DB2
    PB14     ------> DB1
    PB13     ------> DB0
    PB12     ------> E
    PB11     ------> RW
    PB10     ------> RS
    GPIO Configuration: General purpose output push-pull*/
    
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);
    
    // GPIOA Configuration
    // 00: General purpose output push-pull
    CLEAR_BIT(GPIOA->CRH, GPIO_CRH_CNF8);
    CLEAR_BIT(GPIOA->CRH, GPIO_CRH_CNF9);
    CLEAR_BIT(GPIOA->CRH, GPIO_CRH_CNF10);
    CLEAR_BIT(GPIOA->CRH, GPIO_CRH_CNF11);
    CLEAR_BIT(GPIOA->CRH, GPIO_CRH_CNF12);
    
    // 01: Output mode, max speed 10 MHz.
    CLEAR_BIT(GPIOA->CRH, GPIO_CRH_MODE8);
    SET_BIT(GPIOA->CRH, GPIO_CRH_MODE8_0);
    CLEAR_BIT(GPIOA->CRH, GPIO_CRH_MODE9);
    SET_BIT(GPIOA->CRH, GPIO_CRH_MODE9_0);
    CLEAR_BIT(GPIOA->CRH, GPIO_CRH_MODE10);
    SET_BIT(GPIOA->CRH, GPIO_CRH_MODE10_0);
    CLEAR_BIT(GPIOA->CRH, GPIO_CRH_MODE11);
    SET_BIT(GPIOA->CRH, GPIO_CRH_MODE11_0);
    CLEAR_BIT(GPIOA->CRH, GPIO_CRH_MODE12);
    SET_BIT(GPIOA->CRH, GPIO_CRH_MODE12_0);
    
    // GPIOB Configuration
    // 00: General purpose output push-pull
    CLEAR_BIT(GPIOB->CRH, GPIO_CRH_CNF10);
    CLEAR_BIT(GPIOB->CRH, GPIO_CRH_CNF11);
    CLEAR_BIT(GPIOB->CRH, GPIO_CRH_CNF12);
    CLEAR_BIT(GPIOB->CRH, GPIO_CRH_CNF13);
    CLEAR_BIT(GPIOB->CRH, GPIO_CRH_CNF14);
    CLEAR_BIT(GPIOB->CRH, GPIO_CRH_CNF15);
    
     // 01: Output mode, max speed 10 MHz.
    CLEAR_BIT(GPIOB->CRH, GPIO_CRH_MODE10);
    SET_BIT(GPIOB->CRH, GPIO_CRH_MODE10_0);
    CLEAR_BIT(GPIOB->CRH, GPIO_CRH_MODE11);
    SET_BIT(GPIOB->CRH, GPIO_CRH_MODE11_0);
    CLEAR_BIT(GPIOB->CRH, GPIO_CRH_MODE12);
    SET_BIT(GPIOB->CRH, GPIO_CRH_MODE12_0);
    CLEAR_BIT(GPIOB->CRH, GPIO_CRH_MODE13);
    SET_BIT(GPIOB->CRH, GPIO_CRH_MODE13_0);
    CLEAR_BIT(GPIOB->CRH, GPIO_CRH_MODE14);
    SET_BIT(GPIOB->CRH, GPIO_CRH_MODE14_0);
    CLEAR_BIT(GPIOB->CRH, GPIO_CRH_MODE15);
    SET_BIT(GPIOB->CRH, GPIO_CRH_MODE15_0);
}