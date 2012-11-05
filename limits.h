/*
  limits.h - code pertaining to limit-switches and performing the homing cycle
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#ifndef limits_h
#define limits_h 

#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<X2_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits
#define HOME_MASK ((1<<X_LIMIT_BIT)|(1<<X2_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

#define HOME_EVENTS_PER_SECOND (10000L)
#define INDEP_EVENT_COUNT (1L<<30)

extern uint8_t potential_hardlimit;

// initialize the limits module
void limits_init();
void home_init();

// perform the homing cycle
void limits_go_home();

// read home & limit switch status
uint8_t home_limit_state();

// This function is used inside the Stepper Driver Interrupt when homing
bool indep_increment(indep_t_ptr ht);

#endif