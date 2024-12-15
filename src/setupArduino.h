//Simeon Dimitrov
//Grace Cryer
//Keiran Cole
//This file serves to define functions that would typically be available in the Arduino library

#ifndef HELPERS_H
#define HELPERS_H

#include <Arduino.h>

#define RDA 0x80
#define TBE 0x20  

void adcInit();
unsigned int adc_read(unsigned char adc_channel_num);

void U0Init(int U0baud);
unsigned char U0kbhit();
unsigned char U0getchar();
void U0putchar(unsigned char U0pdata);
void U0printString(const char *str);
void U0printNumber(int n);
void U0printFloat(float f);

void my_delay(unsigned int freq);

#endif
