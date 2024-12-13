#include "myArduino.h"

void adcInit() {
    // setup the A register
    ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
    ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
    ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
    ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
    // setup the B register
    ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
    ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
    // setup the MUX Register
    ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
    ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
    ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
    ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num) {
    // clear the channel selection bits (MUX 4:0)
    ADMUX  &= 0b11100000;
    // clear the channel selection bits (MUX 5)
    ADCSRB &= 0b11110111;
    // set the channel number
    if (adc_channel_num > 7) {
        // set the channel selection bits, but remove the most significant bit (bit 3)
        adc_channel_num -= 8;
        // set MUX bit 5
        ADCSRB |= 0b00001000;
    }
    // set the channel selection bits
    ADMUX  += adc_channel_num;
    // set bit 6 of ADCSRA to 1 to start a conversion
    ADCSRA |= 0x40;
    // wait for the conversion to complete
    while ((ADCSRA & 0x40) != 0);
    // return the result in the ADC data register
    return ADC;
}

void U0Init(int U0baud) {
    unsigned long FCPU = 16000000;
    unsigned int tbaud;
    tbaud = (FCPU / 16 / U0baud - 1);
    // Same as (FCPU / (16 * U0baud)) - 1;
    UCSR0A = 0x20;
    UCSR0B = 0x18;
    UCSR0C = 0x06;
    UBRR0  = tbaud;
}

unsigned char U0kbhit() {
    return UCSR0A & RDA;
}

unsigned char U0getchar() {
    return UDR0;
}

void U0putchar(unsigned char U0pdata) {
    while((UCSR0A & TBE)==0);
    UDR0 = U0pdata;
}

void U0printString(const char *str) {
    int i = 0;
    while(str[i] != '\0') {
        U0putchar(str[i]);
        i++;
    }
}

void U0printNumber(int n) {
    String nstr = String(n);  
    U0printString(nstr.c_str()); 
}

void U0printFloat(float f) {
    char temp[32]; 
    dtostrf(f, 4, 2, temp); 
    U0printString(temp); 
}

void my_delay(unsigned int freq) {
    // calc period
    double period = 1.0/double(freq);
    // 50% duty cycle
    double half_period = period/ 2.0f;
    // clock period def
    double clk_period = 0.0000000625;
    // calc ticks 
    unsigned int ticks = half_period / clk_period;
    // stop the timer
    TCCR1B &= 0xF8;
    // set the counts
    TCNT1 = (unsigned int) (65536 - ticks);
    // start the timer
    TCCR1A = 0x0;
    TCCR1B |= 0b00000001;
    // wait for overflow
    while((TIFR1 & 0x01)==0); // 0b 0000 0000
    // stop the timer
    TCCR1B &= 0xF8;   // 0b 0000 0000
    // reset TOV           
    TIFR1 |= 0x01;
}
