/*
  stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Sungeun K. Jeon
  Copyright (c) 2012 Chuck Harrison for http://opensourceecology.org/wiki/CNC_Torch_Table

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

#ifndef stepper_h
#define stepper_h 

#include <avr/io.h>
#include <avr/sleep.h>
#include <stdbool.h>

typedef struct indep_t *indep_t_ptr;

// Some useful constants
#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits
#define STEPPING_MASK (STEP_MASK | DIRECTION_MASK) // All stepping-related bits (step/direction)

#define STEPPERS_DISABLE_MASK ((1<<STEPPERS_DISABLE_BIT)|(1<<X2_DISABLE_BIT))


// Initialize and setup the stepper motor subsystem
void st_init();

// Enable steppers, but cycle does not start unless called by motion control or runtime command.
void st_wake_up();

// Immediately disables steppers
void st_go_idle();

// Reset the stepper subsystem variables       
void st_reset();
             
// Notify the stepper subsystem to start executing the g-code program in buffer.
void st_cycle_start();

// Reinitializes the buffer after a feed hold for a resume.
void st_cycle_reinitialize(); 

// Initiates a feed hold of the running program
void st_feed_hold();

// Start an independent-axis move
void st_indep_start(indep_t_ptr frame);

void inline disable_steppers();

extern uint8_t out_bits0;
extern bool indep_mode;

// contains variables for one axis' independent trapezoidal movement
// definitions for indep_t.flags
#define INDEP_HOMING (0x01)
#define INDEP_NO_TARGET (0x02)
#define INDEP_HIT_HOME (0x04)
struct indep_t {
  int32_t target_pos;
  int32_t decel_pos;
  int32_t dpdt; // rate, st.counter_n LSB's/tick
  int32_t d2pdt2a; // accel, st.counter_n LSB's/tick^2
  int32_t d2pdt2d; // decel, st.counter_n LSB's/tick^2
  int32_t dpdt_max;
  int32_t dpdt_min;
  uint8_t home_mask;
  uint8_t home_nullstate;
  uint8_t axis;
  enum indep_state {
    idle,
    accel,
    slew,
    decel,
    done,
    fault
  } state;
  uint8_t flags;
  indep_t_ptr next_axis; // indep_t*
  indep_t_ptr prev_axis; // indep_t*
};

#endif
