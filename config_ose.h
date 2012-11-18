/*
  config_ose.h - compile time configuration, specific to OSE steppernug systems
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011-2012 Sungeun K. Jeon
  Copyright (c) 2012 Chuck Harrison
  

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

// included from bottom of config.h

// I2C I/O Expander
#define MCP23017_PRESENT
#define MCP23017_ADDR   0x20  // 7-bit address, lsbits are A2..A0

// Define pin-assignments
#undef STEPPING_DDR
#undef STEPPING_PORT

#define STEPPING_DDR       DDRB
#define STEPPING_PORT      PORTB

#undef X_STEP_BIT 
#undef Y_STEP_BIT
#undef Z_STEP_BIT
#undef X_DIRECTION_BIT
#undef Y_DIRECTION_BIT
#undef Z_DIRECTION_BIT
#define X_STEP_BIT         5  // Uno Digital Pin 13
#define Y_STEP_BIT         3  // Uno Digital Pin 11
#define Z_STEP_BIT         1  // Uno Digital Pin 9
#define X_DIRECTION_BIT    4  // Uno Digital Pin 12
#define Y_DIRECTION_BIT    2  // Uno Digital Pin 10
#define Z_DIRECTION_BIT    0  // Uno Digital Pin 8
#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits
#define STEPPING_MASK (STEP_MASK | DIRECTION_MASK) // All stepping-related bits (step/direction)

#undef STEPPERS_DISABLE_DDR
#undef STEPPERS_DISABLE_PORT
#undef STEPPERS_DISABLE_BIT
#undef STEPPERS_DISABLE_MASK
#undef STEPPERS_DISABLE_INVERT_MASK
#define STEPPERS_DISABLE_DDR    DDRD
#define STEPPERS_DISABLE_PORT   PORTD
#define STEPPERS_DISABLE_BIT    4  // Uno Digital Pin 4
#define X2_DISABLE_BIT          3  // Uno Digital Pin 3  Hi disables X2
#define STEPPERS_DISABLE_MASK ((1<<STEPPERS_DISABLE_BIT)|(1<<X2_DISABLE_BIT))

#define STEPPERS_DISABLE_INVERT_MASK  (1<<STEPPERS_DISABLE_BIT) // Logic low out for disable (e.g. steppernug)

#undef LIMIT_DDR
#undef LIMIT_PIN
#undef LIMIT_PORT
#undef X_LIMIT_BIT
#undef Y_LIMIT_BIT
#undef Z_LIMIT_BIT
#undef LIMIT_MASK
#undef HOME_MASK

#define USE_I2C_LIMITS

#ifdef USE_I2C_LIMITS
#undef LIMIT_INT

#define X2_LIMIT_BIT  2  // Uno Digital Pin 2 (INT0)
#define X_LIMIT_BIT   7  // Uno Digital Pin 7
#define Y_LIMIT_BIT   6  // Uno Digital Pin 6
#define Z_LIMIT_BIT   5  // Uno Digital Pin 5
#define X2_LIMIT_BIT  2  // Uno Digital Pin 2 (INT0)
#define X_HOME_BIT   7  // Uno Digital Pin 7
#define Y_HOME_BIT   6  // Uno Digital Pin 6
#define Z_HOME_BIT   5  // Uno Digital Pin 5
#define X2_HOME_BIT  2  // Uno Digital Pin 2 (INT0)

#else
#define X2_LIMIT_BIT  2  // Uno Digital Pin 2 (INT0)
#define LIMIT_DDR     DDRD
#define LIMIT_PIN     PIND
#define LIMIT_PORT    PORTD
#define X_LIMIT_BIT   7  // Uno Digital Pin 7
#define Y_LIMIT_BIT   6  // Uno Digital Pin 6
#define Z_LIMIT_BIT   5  // Uno Digital Pin 5
#define X2_LIMIT_BIT  2  // Uno Digital Pin 2 (INT0)
#define HOME_PIN      PIND
#define X_HOME_BIT   7  // Uno Digital Pin 7
#define Y_HOME_BIT   6  // Uno Digital Pin 6
#define Z_HOME_BIT   5  // Uno Digital Pin 5
#define X2_HOME_BIT  2  // Uno Digital Pin 2 (INT0)

#endif
#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<X2_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits
#define HOME_MASK ((1<<X_HOME_BIT)|(1<<X2_HOME_BIT)|(1<<Y_HOME_BIT)|(1<<Z_HOME_BIT)) // All limit bits
#define LIMITS_INVERT_MASK 0

#undef SPINDLE_PRESENT
#undef COOLANT_PRESENT
