/*************************************************** 
  This is a limited-function library for the MCP23017 i2c port expander

  (c) 2012-13 Chuck Harrison for http:/opensourceecology.org
  BSD license
  (however check license of twi.c against which it links)
  
  Inspired by Adafruit_MCP23017.c whose copyright notice appears below
  > Adafruit invests time and resources providing this open source code, 
  > please support Adafruit and open-source hardware by purchasing 
  > products from Adafruit!
  > 
  > Written by Limor Fried/Ladyada for Adafruit Industries.  
  > BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "twi.h"
#include <avr/pgmspace.h>
#include "MCP23017.h"
#include "config.h"
#include <avr/interrupt.h>


////////////////////////////////////////////////////////////////////////////////
void init_MCP23017_interrupt(); // forward declaration

void MCP23017_begin(uint8_t addr) {
  i2caddr = MCP23017_ADDRESS | (addr&7);
  twi_init();
  twi_releaseBus();
  // set defaults
  // All  pins input at startup (some B pins will become outputs later, e.g for spindle control)
  static uint8_t localbuf[3];
  localbuf[0]=MCP23017_IODIRA; localbuf[1]=0xFF; localbuf[2]=0xFF;
  twi_writeTo(i2caddr, localbuf, 3, DO_WAIT);
  #ifdef USE_I2C_LIMITS
  // set up IOCON.SEQOP=0, BANK=0 
  //  also INT output is active low, active driver.
  localbuf[0]=MCP23017_IOCONA; localbuf[1]=0x00; // IOCON.SEQOP=1: 0x20; 
  twi_writeTo(i2caddr, localbuf, 2, DO_WAIT);
  // set up INTCONA for interrupt on change (same as power-on default)
  localbuf[0]=MCP23017_INTCONA; localbuf[1]=0x00; 
  twi_writeTo(i2caddr, localbuf, 2, DO_WAIT);
  // set up GPINTENA to enable interrupt on all pins
  localbuf[0]=MCP23017_GPINTENA; localbuf[1]=0xFF; 
  twi_writeTo(i2caddr, localbuf, 2, DO_WAIT);
  init_MCP23017_interrupt();
  #endif
}



void MCP23017_pinMode(uint8_t p, uint8_t d) {
  uint8_t localbuf[2];

  // only 16 bits!
  if (p > 15)
    return;

  if (p < 8)
    localbuf[0] = MCP23017_IODIRA;
  else {
    localbuf[0] = MCP23017_IODIRB;
    p -= 8;
  }

  // read the current IODIR
  if(twi_writeTo(i2caddr, localbuf, 1, DO_WAIT) != 0) return;
  if(twi_readFrom(i2caddr, &localbuf[1], 1) != 1) return;
  // set the pin and direction
  if (d == INPUT) {
    localbuf[1] |= 1 << p; 
  } else {
    localbuf[1] &= ~(1 << p);
  }

  // write the new IODIR
  twi_writeTo(i2caddr, localbuf, 2, DO_WAIT);
}

uint16_t MCP23017_readGPIOAB() {
  uint16_t ba = 0;
  
  uint8_t localbuf[3] = {MCP23017_GPIOA, 0, 0};

  // read the current GPIO output latches
  if(twi_writeTo(i2caddr, localbuf, 1, DO_WAIT) != 0) return 0;
  if(twi_readFrom(i2caddr, &localbuf[1], 2) != 2) return 0;

  ba = localbuf[2];
  ba <<= 8;
  ba |= localbuf[1];

  return ba;
}

void MCP23017_digitalWrite(uint8_t p, uint8_t d) {
  uint8_t localbuf[2];
  uint8_t olataddr;

  // only 16 bits!
  if (p > 15)
    return;

  if (p < 8) {
    olataddr = MCP23017_OLATA;
    localbuf[0] = MCP23017_GPIOA;
  } else {
    olataddr = MCP23017_OLATB;
    localbuf[0] = MCP23017_GPIOB;
    p -= 8;
  }

  // read the current GPIO output latches
  uint8_t status = twi_writeTo(i2caddr, &olataddr, 1, DO_WAIT);
  if(twi_readFrom(i2caddr, &localbuf[1], 1) != 1) return;
  // set the pin 
  if (d != 0) {
    localbuf[1] |= 1 << p; 
  } else {
    localbuf[1] &= ~(1 << p);
  }
  // write the new GPIO
  status = twi_writeTo(i2caddr, localbuf, 2, DO_WAIT);
}

#ifdef MCP23017_INT_PIN // if defined, it is 0 or 1
// Use MCP23017's interrupt output to trigger a GPIO read operation
// Using AVR's dedicated interrupts INT0/INT1 (not pin-change interrupt)
uint8_t GPIO_read_buf[2];
twi_transaction_read GPIOread_trans;
void init_MCP23017_interrupt() {
  GPIOread_trans.address = i2caddr;
  GPIOread_trans.reg = MCP23017_GPIOA;
  GPIOread_trans.length = 1;
  GPIOread_trans.data = GPIO_read_buf;
  // set INTx falling edge sensitive
  // TBD: may need to be level sensitive in case we get out of sync with MCP23017
  EICRA = (EICRA & ~( 3 << (2*MCP23017_INT_PIN) )) | ( 2 << (2*MCP23017_INT_PIN) );
  EIMSK |= 1 << (MCP23017_INT_PIN);
}  
ISR(MCP23017_INT_vect) 
{
  // schedule a read operation at priority 0
  twi_queue_read_transaction(&GPIOread_trans, 0);
}
#else
void init_MCP23017_interrupt() { }
#endif

