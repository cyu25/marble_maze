// From E. Williams, Make: AVR Programming, Chapter 17
// Comments, small mods by E.W. Hansen, 2 Feb 2019
// Changed "initI2C" to "i2cInit" for consistency (EWH, 3 Feb 2021)

#include "i2c.h"
#include "ioE28.h"      	// tiny_printf as well as some reading

void i2cInit(void) {
	// Bit rate = F_CPU / (16 + 2*TWBR * Prescaler)
	//  => TWBR = ((F_CPU / Bit rate) - 16) / (2*Prescaler)
	//  => TWBR = (16e6 / 100e3 - 16) / 2 = 72 for 100 kHz with 16 MHz clock
	TWBR = 72;                       

	// TWI enable (TWEN), ACK enable (TWEA), per Arduino Wires.h
  	TWCR = (1<<TWEN) | (1<<TWEA);			
}

void i2cWaitForComplete(void) {
// Loop until hardware sets interrupt flag (TWINT)
//printf("wait1\n\r");
	while( !(TWCR & (1 << TWINT)) );
 // printf("wait2\n\r");
}

void i2cStart(void) {
// Enable the TWI (TWEN), request master start (TWSTA), clear done flag (TWINT)
  //printf("i2cStart\n\r");
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
  i2cWaitForComplete();
}

void i2cStop(void) {
// Enable the TWI (TWEN), generate master stop (TWSTO), clear done flag (TWINT)
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

uint8_t i2cReadAck(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) |  (1 << TWEA);
  i2cWaitForComplete();
  return (TWDR);
}

uint8_t i2cReadNoAck(void) {
  TWCR = (1 << TWINT) | (1 << TWEN);
  i2cWaitForComplete();
  return (TWDR);
}

void i2cSend(uint8_t data) {
  TWDR = data;
  TWCR = (1 << TWINT) | (1 << TWEN);                  /* init and enable */
  i2cWaitForComplete();
}
