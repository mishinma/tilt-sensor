/**
  ******************************************************************************
  * file:     atan_lut.c
  * author:   Mikhail Mishin
  * version:  V1.0
  * date:     15-May-2016
  * brief:    This file provides implementation of the `atan2_lut` function using
              a look-up table and linear interpolation
  ******************************************************************************
                                                                              */

/* Includes ------------------------------------------------------------------*/

#include "tilt_sensor.h"
#include <stdlib.h>

int16_t atan_lut(int8_t, int8_t);
uint16_t lookup_angle(uint16_t);
int sign(int);



const uint16_t LUT[102] = {
    0, 10, 20, 30, 40, 50, 60, 70, 80, 90,
    100, 110, 119, 129, 139, 149, 159, 168, 178, 188,
    197, 207, 217, 226, 236, 245, 254, 264, 273, 282,
    291, 301, 310, 319, 328, 337, 346, 354, 363, 372,
    381, 389, 398, 406, 415, 423, 431, 439, 448, 456,
    464, 472, 480, 487, 495, 503, 510, 518, 526, 533,
    540, 548, 555, 562, 569, 576, 583, 590, 597, 604,
    611, 617, 624, 631, 637, 644, 650, 656, 662, 669,
    675, 681, 687, 693, 699, 704, 710, 716, 722, 727,
    733, 738, 744, 749, 754, 760, 765, 770, 775, 780,
    785, 785};

/**
 * brief  Computes atan2(y/x)
 *
 */ 
int16_t atan2_lut(int8_t x, int8_t y){
    int16_t angle;
    if (x > 0){
        angle = atan_lut(x, y);
    }
    else if (x < 0){
        if (y >= 0){
            angle = PI + atan_lut(x, y);
        }
        else{
            angle = -PI + atan_lut(x, y);
        }
    }
    // x = 0
    else {
        if (y > 0){
            angle = PI_HALF;
        }
        else if (y < 0){
            angle = -PI_HALF;
        }
        else {
            angle = 0;
        }
    }
    return angle;
}


/**
 * brief  Computes atan(y/x)
 *
 */ 
int16_t atan_lut(int8_t x, int8_t y){
    uint16_t ratio;
    int16_t angle;
    int8_t prod_sign;
    prod_sign = sign(x)*sign(y);
    x = abs(x);
    y = abs(y);
    if (y <= x){
        ratio = y*1000/x;
        angle = lookup_angle(ratio);
    }
    else{
        ratio = x*1000/y;
        angle = PI_HALF - lookup_angle(ratio);
    }
    if (prod_sign < 0){
        angle = -angle;
    }
    return angle;
        
 }
 
 
 /**
 * brief  Look-up values and apply linear interpolation
 *
 */ 
uint16_t lookup_angle(uint16_t input){
    uint8_t index, r_input;
    uint16_t angle;
    r_input = input/10;
    index = r_input;
    angle = LUT[index] + (input - r_input*10)*(LUT[index+1] - LUT[index]);
    return angle;
}

int sign(int x) {
    return (x > 0) - (x < 0);
}
 
int16_t rad2deg(int16_t rad){
    uint32_t temp;
    temp = (abs(rad)*1000 + PI_HALF)/PI;
    temp = temp*180/100;
    return (int16_t) sign(rad)*temp;
}

 