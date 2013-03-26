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

#ifdef SPINDLE_ON_I2C
#include "twi.h"
#include "MCP23017.h"

twi_transaction_write_one_masked trans;

#endif

static uint8_t current_direction;

void spindle_init()
{
  current_direction = 0;
#ifdef SPINDLE_PRESENT
#ifdef SPINDLE_ON_I2C
  trans.address = i2caddr;
  // TBD: uncommenting the write transaction seems to disable MCP23017 INT generation
  //set output direction
  trans.reg = MCP23017_IODIRB;
  trans.data = 0 ;
  trans.mask = (1 << SPINDLE_ENABLE_BIT) | (1 << SPINDLE_DIRECTION_BIT) ;
  //twi_queue_write_one_masked_transaction(&trans, 1);
  // prepare for data
  trans.reg = MCP23017_OLATB;
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
  trans.data = 0;
  trans.mask = 1 << SPINDLE_ENABLE_BIT;
  trans.reg = MCP23017_OLATB;
  twi_queue_write_one_masked_transaction(&trans, 1);
#else
  SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
#endif
#endif
}

// direction is 1, -1, 0 for M3, M4, M5
void spindle_run(int8_t direction) //, uint16_t rpm) 
{
#ifdef SPINDLE_PRESENT
  if (direction != current_direction) {
    plan_synchronize();
    
#ifdef SPINDLE_ON_I2C
    if(direction) {
      if (direction < 0) {
        trans.data = (1 << SPINDLE_ENABLE_BIT) | (1 << SPINDLE_DIRECTION_BIT) ;
      } else {
        trans.data = (1 << SPINDLE_ENABLE_BIT);
      }
      trans.mask = (1 << SPINDLE_ENABLE_BIT) | (1 << SPINDLE_DIRECTION_BIT) ;
      trans.reg = MCP23017_OLATB;
      twi_queue_write_one_masked_transaction(&trans, 1);
      
#else
    if(direction) {
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
