// @file
// @brief Low level stepper routines

#ifndef STEPPER_H
#define STEPPER_H

#include "config.h"
#include <inttypes.h>

// watch stall guard load signal on LEDs in realtime; useful for
// stepper electrical configuraiton tuning
//#define LED_SG_DIAG 1

// Idler stepper is a standard 200 step/1.8 degree unit running with
// 16 microsteps.  The stop reduces total travel from 360 degrees, but
// 3200 steps is a good upper travel bound we should never reach.
#define IDLER_CAL_BARREL_STEPS 3200

// default steps from idler stop to filament 1 engaged position
#define IDLER_CAL_HOMING_STEPS 130

// Retry calibration process five times before asking user for help
#define IDLER_CAL_ATTEMPTS 5

// Check stop repeatability this many times (must be >=2)
#define IDLER_CAL_SAMPLES 3

// Do not accept repeatability worse than .56 degrees
#define IDLER_CAL_TOLERANCE 5

// ten degree backoff in stop repeatability test; we're balancing
// motor resonance, barrel resonance, MMU2S body resonance, startup
// variability, etc
#define IDLER_CAL_BACKOFF_STEPS 90

// timing delay (microseconds) between calibration travel steps; again
// this is a balance between physical inertial factors.
#define IDLER_CAL_DELAY 900

// StallGuard readings are meaningless before 4 full steps; make sure
// we have a running start before watching steps for stall.  Ignore
// this many steps at the beginning of travel before paying attention
// to StallGuard.  64 is the minumum useful value.
#define IDLER_CAL_STALLGUARD_PRESTEPS 64

// Selector leadscrew is 120mm, 8mm travel per revolution. Stepper is
// a 200 step unit running 2 microsteps. .02mm per step == 6000 steps.
// We'd never be able to use all of it (leadnut depth, supports,
// etc, cut into usable length), but in the future we'll try to use as
// much of it as we can and 6000 is a certain absolute upper bound.
// If we don't find an endstop in that many steps, something has
// definitely gone wrong.
#define SELECTOR_CAL_LEADSCREW_STEPS 6000

// Retry selector calibration process five times before asking user
// for help
#define SELECTOR_CAL_ATTEMPTS 5

// Check stop repeatability this many times (must be >=2)
#define SELECTOR_CAL_SAMPLES 3
#define SELECTOR_HOME_SAMPLES 3

// Do not accept repeatability worse than 5 steps * .05 == +/-.1mm
#define SELECTOR_CAL_TOLERANCE 5

// ~ half a cm backoff in endstop repeatability test; we're balancing
// motor resonance, MMU2S body resonance, startup variability and the
// Triaminic doing whatever it does inside its tiny little head.
#define SELECTOR_CAL_BACKOFF_STEPS 100

// timing delay (microseconds) between calibration travel steps; again
// this is a balance between physical inertial factors. 600-700us
// seems to be in the middle of the sweet-spot here. StallGuard in
// config.h MUST be tuned to match.
#define SELECTOR_CAL_DELAY 700

// slow delay is for low-velocity, low-force intentional skipping at
// the stops intended to do far less flexing.  No StallGuard here.
#define SELECTOR_CAL_SLOW_DELAY 2500

// StallGuard readings are noisy garbage when the motor first starts
// stepping; make sure we have a running start before watching steps
// for stall.  Ignore this many steps at the beginning of travel
// before paying attention to StallGuard. 8 is the absolute minimum (4
// full steps == 8 half steps), more works better, make sure it's a
// good bit less than backoff_steps above.
#define SELECTOR_CAL_STALLGUARD_PRESTEPS 32

extern int8_t filament_type[MAX_EXTRUDERS];

void stepper_state_init(void);
int calibrate(bool retry_forever);
int home(bool retry_forever);
uint8_t get_idler_offset();
bool set_idler_offset(uint8_t);
uint8_t get_selector_offset();
bool set_selector_offset(uint8_t);

void engage_idler(bool _unpark);
void select_idler(int _idler);
void reset_idler();
int home_idler();
void reset_selector();
int home_selector();

void select_idler_selector(uint8_t _idler_selector);
void select_idler_selector(uint8_t idler, uint8_t selector);

void feed_to_bondtech();
void unload_to_finda();
void door_sensor_detected();

void set_pulley_dir_push();
void set_pulley_dir_pull();
void do_pulley_step();


#define ERR_IDLER_CAL_JAM          100
#define ERR_IDLER_CAL_MISSING_STOP 101
#define ERR_IDLER_CAL_TOLERANCE    102

#define ERR_SELECTOR_JAM           110
#define ERR_SELECTOR_MISSING_STOP  111

#define ERR_SELECTOR_LEFT_TOLERANCE       112
#define ERR_SELECTOR_LEFT_FLEX_TOLERANCE  113
#define ERR_SELECTOR_RIGHT_TOLERANCE      114
#define ERR_SELECTOR_RIGHT_FLEX_TOLERANCE 115
#define ERR_SELECTOR_DIMENSIONS_FAILED    116
#endif //STEPPER_H

