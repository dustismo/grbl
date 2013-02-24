/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2012 Sungeun K. Jeon

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "settings.h"
#include "spindle_control.h"
#include "planner.h"

static uint8_t current_direction;

void spindle_init()
{
  current_direction = 0;
#ifdef SPINDLE_PRESENT
#ifdef SPINDLE_ON_I2C
// no need to do anything?
#else  
  SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT);
  SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); 
#endif
#endif
  spindle_stop();
}

void spindle_stop()
{
#ifdef SPINDLE_PRESENT
#ifdef SPINDLE_ON_I2C
// uint8_t localbuf[2] = { MCP23017_OLATB, 0 };
// while(-1 == twi_nonBlockingReadRegisterFrom(i2caddr, MCP23017_OLATB, localbuf+1, 1)) { }
// note potential race condition if GPIOB is written by someone else in an interrupt routine
// localbuf[1] &= ~(1<<SPINDLE_ENABLE_BIT);
// twi_writeTo(i2caddr, localbuf, 2, DONT_WAIT);
#else
  SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
#endif
#endif
}

void spindle_run(int8_t direction) //, uint16_t rpm) 
{
#ifdef SPINDLE_PRESENT
  if (direction != current_direction) {
    plan_synchronize();
#ifdef SPINDLE_ON_I2C
//  uint8_t localbuf[2] = { MCP23017_OLATB, 0 };
//  while(-1 == twi_nonBlockingReadRegisterFrom(i2caddr, MCP23017_OLATB, localbuf+1, 1)) { }
//  note potential race condition if GPIOB is written by someone else in an interrupt routine
    if (direction) {
      if(direction > 0) {
//      localbuf[1] &= ~(1<<SPINDLE_DIRECTION_BIT);
      } else {
//      localbuf[1] |= 1<<SPINDLE_DIRECTION_BIT;
      }
//    localbuf[1] |= 1<<SPINDLE_ENABLE_BIT;
//    twi_writeTo(i2caddr, localbuf, 2, DONT_WAIT)

#else
    if (direction) {
      if(direction > 0) {
        SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
      } else {
        SPINDLE_DIRECTION_PORT |= 1<<SPINDLE_DIRECTION_BIT;
      }
      SPINDLE_ENABLE_PORT |= 1<<SPINDLE_ENABLE_BIT;
#endif
    } else {
      spindle_stop();     
    }
    current_direction = direction;
  }
#endif
}
