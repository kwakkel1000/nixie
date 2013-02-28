//
//
// @ Project : nixie
// @ File Name : nixie.cpp
// @ Date : 23-02-2013
// @ Author : Gijs Kwakkel
//
//
// Copyright (c) 2013 Gijs Kwakkel
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
//


#include "aux_globals.h"
#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>

#define NPORT PORTD

#define PPORT PORTB
#define P1 0
#define P2 1
#define P3 2

uint8_t nixiePWM[3];
uint8_t nixieHIGH[3];
uint8_t nixieLOW[3];
uint8_t nixie;
uint8_t nixiePWMindex;

uint8_t nixieDIM = 10;

#include "../communication/include/i2c.h"
#define DS1307 0xD0


//#include "lcd/include/HD44780.h"

//#define I2C_EEPROM_1 0xA0 // needs 8bits not 7bits
//#define SPI_ENC28J60 PB5

//#define VREF 5

// objects
//HD44780 lcd;

// nixie pwm timer
ISR(TIMER0_COMPA_vect)
{
    cli();
    PPORT &= ~((1 << nixiePWM[0]) | (1 << nixiePWM[1]) | (1 << nixiePWM[2]));
    if (nixiePWMindex <= 2)
    {
//        fcpu_delay_us(512 - (64 * nixieDIM));
        fcpu_delay_us(128);
        NPORT = (nixieLOW[nixiePWMindex] << 4) | nixieHIGH[nixiePWMindex];
        PPORT |= (1 << nixiePWM[nixiePWMindex]);
        fcpu_delay_us(128 * nixieDIM);
//        fcpu_delay_us(64 * nixieDIM);
    }
    if (nixiePWMindex >= (12 - nixieDIM))
//    if (nixiePWMindex == 2)
    {
        nixiePWMindex = 0;
    }
    else
    {
        nixiePWMindex++;
    }
//    sei();
}

int tmpTime = 0;
// clock i2c timer
ISR(TIMER1_COMPA_vect)
{
    cli();
    i2c::start();
    if (i2c::selectSlave(DS1307, I2C_WRITE) == SUCCESS)
    {
        i2c::write(0);
    }
    i2c::start();
    if (i2c::selectSlave(DS1307, I2C_READ) == SUCCESS)
    {
        tmpTime = i2c::read(true);
        nixieHIGH[2] = ((tmpTime & 0b01110000) >> 4);
        nixieLOW[2] = (tmpTime & 0b00001111);

        tmpTime = i2c::read(true);
        nixieHIGH[1] = ((tmpTime & 0b01110000) >> 4);
        nixieLOW[1] = (tmpTime & 0b00001111);

        tmpTime = i2c::read(false);
        nixieHIGH[0] = ((tmpTime & 0b00110000) >> 4);
        nixieLOW[0] = (tmpTime & 0b00001111);
    }
    i2c::stop();
    sei();
}

uint16_t readADC(uint8_t ADCchannel)
{
    //select ADC channel with safety mask
    ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
    //single conversion mode
    ADCSRA |= (1<<ADSC);
    // wait until ADC conversion is complete
    while( ADCSRA & (1<<ADSC) );
    return ADC;
}

void clockInit()
{
    bool freshClock = false;
    // I2C
    i2c::masterInit(0x2A, I2C_PS1); // 100khz

    i2c::start();
    if (i2c::selectSlave(DS1307, I2C_WRITE) == SUCCESS)
    {
        i2c::write(0);
    }
    i2c::start();
    if (i2c::selectSlave(DS1307, I2C_READ) == SUCCESS)
    {
        freshClock = i2c::read(false) & 0b10000000;
    }
    i2c::stop();

    if (freshClock)
    {
        i2c::start();
        if (i2c::selectSlave(DS1307, I2C_WRITE) == SUCCESS)
        {
            i2c::write(0); // select seconds
            i2c::write(0); // set CH 0 (enable the clock)
        }
        i2c::stop();
        i2c::start();
        if (i2c::selectSlave(DS1307, I2C_WRITE) == SUCCESS)
        {
            i2c::write(2);          // select hour
            i2c::write(0);          // set 24 mode
        }
        i2c::stop();
        i2c::start();
        if (i2c::selectSlave(DS1307, I2C_WRITE) == SUCCESS)
        {
            i2c::write(7);          // select control
            i2c::write(0b10010010); // set OUT, set SQWE, unset rs1 unset rs0
        }
        i2c::stop();
    }
}

void nixieInit()
{
    nixieDIM = 10;

    PPORT |= (1 << DDB1);
    PPORT |= (1 << DDB1);
    PPORT |= (1 << DDB1);

    NPORT = 0xFF;

    nixiePWM[0] = P1;
    nixiePWM[1] = P2;
    nixiePWM[2] = P3;

    for (uint8_t i = 9; i != 0; i--)
    {
        nixieHIGH[0] = i;
        nixieHIGH[1] = i;
        nixieHIGH[2] = i;

        nixieLOW[0] = i;
        nixieLOW[1] = i;
        nixieLOW[2] = i;
        fcpu_delay_ms(100);
    }
}

void adcInit()
{
    // adc
    // Select Vref=AVcc
    ADMUX |= (1<<REFS0);
    //set prescaller to 128 and enable ADC
    ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
    // adc
}

void timerInit()
{
    // nixie pwm timer
    // setup timer 0 for CTC
    TCCR0B |= (1 << WGM12); // MAX counter = value OCR1A (Mode 4 / CTC)

    //TCCR0B |= 0x01; // prescaler = 1; // TCCR0B |= (1 << CS10);
    //TCCR0B |= 0x02; // prescaler = 8; // TCCR0B |= (1 << CS11);
    TCCR0B |= 0x03; // prescaler = 64; // TCCR0B |= (1 << CS11) | (1 << CS10);
    //TCCR0B |= 0x04; // prescaler = 256; // TCCR0B |= (1 << CS12);
    //TCCR0B |= 0x05; // prescaler = 1024; // TCCR0B |= (1 << CS12) | (1 << CS10);

    // setup period
    // 16000000 / 64 / 200 = 1250
    // 1250 / 3 (3 tubes multiplexed) = 416.666MHz
//    OCR0A = 200; // OCR0A is 8 bit, so max 255
    OCR0A = 200 ; // OCR0A is 8 bit, so max 255

    // trigger interrupt when Timer0 == OCR0A
    TIMSK0 = 1 << OCIE0A;


    // clock timer
    // setup timer 1 for CTC
    TCCR1B |= (1 << WGM12); // MAX counter = value OCR1A (Mode 4 / CTC)

    //TCCR1B |= 0x01; // prescaler = 1; // TCCR1B |= (1 << CS10);
    //TCCR1B |= 0x02; // prescaler = 8; // TCCR1B |= (1 << CS11);
    //TCCR1B |= 0x03; // prescaler = 64; // TCCR1B |= (1 << CS11) | (1 << CS10);
    //TCCR1B |= 0x04; // prescaler = 256; // TCCR1B |= (1 << CS12);
    TCCR1B |= 0x05; // prescaler = 1024; // TCCR1B |= (1 << CS12) | (1 << CS10);

    // setup period
    // 16000000 / 1024 / 15625 = 1
//    OCR1A = 15625; // OCR1A is 16 bit, so max 65535
    OCR1A = 7812; // OCR1A is 16 bit, so max 65535
//    OCR1A = F_CPU / TCCR1B; // OCR1A is 16 bit, so max 65535

    // trigger interrupt when Timer1 == OCR1A
    TIMSK1 = 1 << OCIE1A;
}

int main(void)
{
    clockInit();

    //adcInit();

    timerInit();
    sei();

    nixieInit();

    for(;;)
    {
        fcpu_delay_ms(480);
    }
}
