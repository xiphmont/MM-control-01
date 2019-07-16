#include "stepper.h"
#include "shr16.h"
#include "tmc2130.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <Arduino.h>
#include "main.h"
#include "mmctl.h"
#include "Buttons.h"
#include "permanent_storage.h"
#include "pins.h"
#include "tmc2130.h"

int8_t filament_type[MAX_EXTRUDERS] = {-1, -1, -1, -1, -1, -1, -1, -1};
static bool isIdlerParked = false;

static const int idler_steps_after_homing = -130;

// 14mm of leadscrew travel, minus PETG shrinkage
static const int selector_steps = 697;
// steps from last extruder to right stop; get this from calibration
static int selector_park_steps = -1;

 // 40 degrees of rotation
static const int idler_steps = 355;
static const int idler_parking_steps = (idler_steps / 2) + 40;  // 40


static int set_idler_direction(int _steps);
static int set_selector_direction(int _steps);
static int set_pulley_direction(int _steps);
static void set_idler_dir_down();
static void set_idler_dir_up();
static void move(int _idler, int _selector, int _pulley);

//! @brief Compute steps for selector needed to change filament
//! @param current_filament Currently selected filament
//! @param next_filament Filament to be selected
//! @return selector steps
int get_selector_steps(int current_filament, int next_filament)
{
    if (current_filament == extruders)
    { // coming out of park
        return ((next_filament - extruders + 1) * selector_steps - selector_park_steps);
    }
    else if (next_filament == extruders)
    { // going into park
        return ((extruders - 1 - current_filament) * selector_steps + selector_park_steps);
    }
    else
    { // moving between two filament positions
        return ((next_filament - current_filament) * selector_steps);
    }
}

//! @brief Compute steps for idler needed to change filament
//! @param current_filament Currently selected filament
//! @param next_filament Filament to be selected
//! @return idler steps
int get_idler_steps(int current_filament, int next_filament)
{
    return ((current_filament - next_filament) * idler_steps);
}

void do_pulley_step()
{
    pulley_step_pin_set();
    asm("nop");
    pulley_step_pin_reset();
    asm("nop");
}

// Idler stepper is a standard 200 step/1.8 degree unit running with
// 16 microsteps.  The stop reduces total travel from 360 degrees, but
// 3200 steps is a good upper travel bound we should never reach.
#define IDLER_CAL_BARREL_STEPS 3200

// Retry calibration process five times before asking user for help
#define IDLER_CAL_ATTEMPTS 5

// Check stop repeatability this many times (must be >=2)
#define IDLER_CAL_SAMPLES 2

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

static int idler_cal_guard_move(int steps) {
  int i;
#if LED_SG_DIAG // watch stall guard load signal on LEDs in realtime; useful for tuning
  int j;
  int min=5;
  int max=0;
#endif
  steps = set_idler_direction(steps);
  // Perform steps.  DO NO EXTRA WORK.  No LEDs, no chatting with the
  // other stepper drivers. If we miss a step due to too much timing
  // jitter, the Triaminic will interpret that as a stall same as
  // hitting an end stop.
  for (i = 0; i < steps; i++) {
    idler_step_pin_set();
    delayMicroseconds(IDLER_CAL_DELAY/2);
    idler_step_pin_reset();
    delayMicroseconds(IDLER_CAL_DELAY/2);
    // Read StallGuard register.  Keep the read in the timing
    // loop even if we're going to disregard it due to startup.
    int sg = tmc2130_read_sg(AX_IDL);
    if (i>IDLER_CAL_STALLGUARD_PRESTEPS && sg==0){
      break;
    }
#if LED_SG_DIAG // diagnostic to display useful realtime stallguard data on LEDs
    uint16_t led=0;
    sg = (sg >> 5) | (sg?1:0);
    for(j=0;j<5;j++)
      led |= (sg>>j?GREEN<<2*(5-j):0);
    if(i>64){
      if(min>sg) min = sg;
      if(min<0) min=0;
      if(max<sg) max = sg;
      if(max>5)max=5;
      led |= ORANGE << 2*(5-min);
      led |= GREEN << 2*(5-max);
      shr16_set_led(led);
    }
#endif
  }
  return i;
}

//! @brief calibrate idler to stop, then move to idler 1 park
//! position.  This is a fast enough process, we use it for homing as
//! well.
//! @retval true Succeeded
//! @retval false Failed
bool home_idler() {
  int i;
  int steps;
  int attempt = 0;

  tmc2130_init(HOMING_MODE);

  // try the calibration process five times before asking for help
  while(1){
    int32_t flex = 0;
    attempt++;

    // how many times have we already tried and failed?  Indicate progress outside stepper loop.
    uint16_t led = 0;
    for (i = 0; i < attempt; i++) led |= ORANGE << 2*(5-i);
    shr16_set_led(led);

    // go to stop
    steps = idler_cal_guard_move(IDLER_CAL_BARREL_STEPS);
    if (steps == IDLER_CAL_BARREL_STEPS) {
      // Did not detect stop.  That's a fail.
      if(attempt == IDLER_CAL_ATTEMPTS) {
        // this is the fifth try, get help from the user.


        // reset the attempt counter, then....
        attempt=0;
      }
      // try again
      continue;
    }

    // verify idler is really at the stop and not just stuck
    // or stuttering.

    // The MMU2s is flexy and the Triaminic is always adjusting its
    // drive behavior, so we need to take that into account.  Hit the
    // stop a few times from the same short distance and see if we get
    // a repeatable number of steps.
    int tries[IDLER_CAL_SAMPLES];
    bool fail_out = 0;
    for(i=0; i < IDLER_CAL_SAMPLES; i++){
      steps = idler_cal_guard_move(-IDLER_CAL_BACKOFF_STEPS);
      if(steps < IDLER_CAL_BACKOFF_STEPS){
        // Jammed? We shouldn't have a short count. Immediately fail and ask for help.


        // then reset attempt counter and continue
        attempt = 0;
        fail_out = 1;
        break;
      }

      // Hit the stop again and note the number of steps.
      steps = idler_cal_guard_move(IDLER_CAL_BACKOFF_STEPS*2);
      if(steps == IDLER_CAL_BACKOFF_STEPS*2){
        // did not detect stop.  That's a fail.
        if(attempt == IDLER_CAL_ATTEMPTS) {
          // This is the fifth try, get help from the user.


          // reset the attempt counter, then....
          attempt=0;
        }
        // try again
        fail_out = 1;
        break;
      }
      tries[i] = steps;
      flex += steps;
    }
    if(fail_out) continue; // propagate fail from inner loop

    // are all our tries within tolerance?
    flex /= IDLER_CAL_SAMPLES;  // now the average
    fail_out = 0;
    for(i=0; i<IDLER_CAL_SAMPLES; i++){
      steps = flex - tries[i];
      if (steps < -IDLER_CAL_TOLERANCE || steps > IDLER_CAL_TOLERANCE){
        // out of bounds.  fail.
        if(attempt == IDLER_CAL_ATTEMPTS) {
          // We're out of bounds and this is the fifth try.  Get help from the user.


          // reset the attempt counter, then....
          attempt=0;
        }
        // try again
        fail_out = 1;
        break;
      }
    }
    if(fail_out) continue; // propagate fail from inner loop

    // SUCCESS! We've passed the stop repeatability test.
    flex -= IDLER_CAL_BACKOFF_STEPS; // now the average deviation from request
    // move to starting position: parked at filament 1
    idler_cal_guard_move(idler_steps_after_homing-(idler_parking_steps + flex));
    isIdlerParked = true;

    tmc2130_init(tmc2130_mode);

    return true;
  }
}

// Selector leadscrew is 120mm, 8mm travel per revolution. Stepper is
// a 200 step unit running 2 microsteps. .02mm per step == 6000 steps.
// We'd never be able to use all of it (leadnut depth, shaft supports,
// etc, cut into usable length), but it's an absolute upper bound.  If
// we don't find an endstop in that many steps, something has
// definitely gone wrong.
#define SELECTOR_CAL_LEADSCREW_STEPS 6000

// Retry selector calibration process five times before asking user
// for help
#define SELECTOR_CAL_ATTEMPTS 5

// Check stop repeatability this many times (must be >=2)
#define SELECTOR_CAL_SAMPLES 2

// Do not accept repeatability worse than +/-.1mm
#define SELECTOR_CAL_TOLERANCE 5

// ~ half a cm backoff in endstop repeatability test; we're balancing
// motor resonance, MMU2S body resonance, startup variability and the
// Triaminic adjusting current over time.
#define SELECTOR_CAL_BACKOFF_STEPS 120

// timing delay (microseconds) between calibration travel steps; again
// this is a balance between physical inertial factors. 600-700us
// seems to be in the middle of the sweet-spot here.
#define SELECTOR_CAL_DELAY 600

// StallGuard readings are noisy garbage when the motor first starts
// stepping; make sure we have a running start before watching steps
// for stall.  Ignore this many steps at the beginning of travel
// before paying attention to StallGuard. 4 full steps (8 half steps)
// is absolute minimum, more works better.
#define SELECTOR_CAL_STALLGUARD_PRESTEPS 32

static int selector_cal_guard_move(int steps) {
  int i;
#if LED_SG_DIAG // watch stall guard load signal on LEDs in realtime; useful for tuning
  int j;
  int min=5;
  int max=0;
#endif
  steps = set_selector_direction(steps);
  // Perform steps.  DO NO EXTRA WORK.  No LEDs, no chatting with the
  // other stepper drivers. If we miss a step due to too much timing
  // jitter, the Triaminic will interpret that as a stall same as
  // hitting an end stop.
  for (i = 0; i < steps; i++) {
    selector_step_pin_set();
    delayMicroseconds(SELECTOR_CAL_DELAY/2);
    selector_step_pin_reset();
    delayMicroseconds(SELECTOR_CAL_DELAY/2);
    // Read StallGuard register.  Keep the read in the timing
    // loop even if we're going to disregard it due to startup.
    uint16_t sg = tmc2130_read_sg(AX_SEL);
    if (i>SELECTOR_CAL_STALLGUARD_PRESTEPS && sg==0){
      break;
    }
#if  LED_SG_DIAG // diagnostic to display useful realtime stallguard data on LEDs
    uint16_t led=0;
    sg = (sg >> 5) | (sg?1:0);
    for(j=0;j<5;j++)
      led |= (sg>>j?GREEN<<2*(5-j):0);
    if(i>64){
      if(min>(int)sg) min = sg;
      if(min<0) min=0;
      if(max<(int)sg) max = sg;
      if(max>5)max=5;
      led |= ORANGE << 2*(5-min);
      led |= GREEN << 2*(5-max);
      shr16_set_led(led);
    }
#endif
  }
  return i;
}

// Side effects galore!  This determines positions of endstops,
// intuits number of extruders from that, saves it all to permanent
// storage, and then homes the selector.

bool calibrate_selector() {
  int i;
  int steps;
  int attempt = 0;

  // if FINDA is sensing filament do not home; indicates filament
  // present, gives user chance to recover by pressing right button
  check_filament_not_present();

  // Now we can attempt calibration.

  tmc2130_init(HOMING_MODE);   // Same as normal mode right now, might not always be

  // full calibration: left, right, and span
  // try the entire process five times before asking for help
  while(1){
    int32_t raw_span = 0;
    int32_t right_flex = 0;
    int32_t left_flex = 0;
    attempt++;

    // how many times have we already tried and failed?  Indicate progress outside stepper loop.
    uint16_t led = 0;
    for (i = 0; i < attempt; i++) led |= ORANGE << 2*(5-i);
    shr16_set_led(led);

    // Go directly to right stop
    steps = selector_cal_guard_move(SELECTOR_CAL_LEADSCREW_STEPS);
    if (steps == SELECTOR_CAL_LEADSCREW_STEPS) {
      // Dd not detect right stop.  That's a fail.
      if(attempt == SELECTOR_CAL_ATTEMPTS) {
        // this is the fifth try, get help from the user.


        // reset the attempt counter, then....
        attempt=0;
      }
      // try again
      continue;
    }

    // verify selector is really at the right stop and not just stuck
    // or stuttering.

    // The MMU2s selector train is flexy and the Triaminic is always
    // adjusting its drive behavior, so we need to take that into
    // account.  Hit the stop a few times from the same short distance
    // and see if we get a repeatable number of steps.
    int tries[SELECTOR_CAL_SAMPLES];
    bool fail_out = 0;
    for(i=0; i < SELECTOR_CAL_SAMPLES; i++){
      steps = selector_cal_guard_move(-SELECTOR_CAL_BACKOFF_STEPS);
      if(steps < SELECTOR_CAL_BACKOFF_STEPS){
        // Jammed? We shouldn't have a short count. Immediately fail and ask for help.



        // then reset attempt counter and continue
        attempt = 0;
        fail_out = 1;
        break;
      }

      // Hit the right stop again and note the number of steps.
      steps = selector_cal_guard_move(SELECTOR_CAL_BACKOFF_STEPS*2);
      if(steps == SELECTOR_CAL_BACKOFF_STEPS*2){
        // did not detect stop.  That's a fail.
        if(attempt == SELECTOR_CAL_ATTEMPTS) {
          // This is the fifth try, get help from the user.


          // reset the attempt counter, then....
          attempt=0;
        }
        // try again
        fail_out = 1;
        break;
      }
      tries[i] = steps;
      right_flex += steps;
    }
    if(fail_out) continue; // propagate fail from inner loop

    // are all our tries within ~.1mm?
    right_flex /= SELECTOR_CAL_SAMPLES;  // now the average
    fail_out = 0;
    for(i=0; i<SELECTOR_CAL_SAMPLES; i++){
      steps = right_flex - tries[i];
      if (steps < -SELECTOR_CAL_TOLERANCE || steps > SELECTOR_CAL_TOLERANCE){
        // out of bounds.  fail.
        if(attempt == SELECTOR_CAL_ATTEMPTS) {
          // We're out of bounds and this is the fifth try.  Get help from the user.


          // reset the attempt counter, then....
          attempt=0;
        }
        // try again
        fail_out = 1;
        break;
      }
    }
    if(fail_out) continue; // propagate fail from inner loop
    right_flex -= SELECTOR_CAL_BACKOFF_STEPS; // now the average deviation

    // We've passed the right stop doublecheck.  Now go to left stop.
    steps = selector_cal_guard_move(-SELECTOR_CAL_LEADSCREW_STEPS);
    if (steps == SELECTOR_CAL_LEADSCREW_STEPS) {
      // did not detect left stop.  That's a fail.
      if(attempt == SELECTOR_CAL_ATTEMPTS) {
        // This is the fifth try, get help from the user.


        // reset the attempt counter, then....
        attempt=0;
      }
      // try again
      continue;
    }

    // We provisionally have our full span measurement: still need to
    // verify left stop repeatability and adjust for flex.
    raw_span = steps;

    // verify selector at left stop and measure flex
    fail_out = 0;
    for(i=0; i < SELECTOR_CAL_SAMPLES; i++){
      steps = selector_cal_guard_move(SELECTOR_CAL_BACKOFF_STEPS);
      if(steps < SELECTOR_CAL_BACKOFF_STEPS){
        // Jammed? Fail and ask for help.



        // then reset attempt counter and continue
        fail_out = 1;
        attempt=0;
        break;
      }

      // Hit the left stop again and note number of steps.
      steps = selector_cal_guard_move(-SELECTOR_CAL_BACKOFF_STEPS*2);
      if(steps == SELECTOR_CAL_BACKOFF_STEPS*2){
        // did not detect left stop.  That's a fail.
        if(attempt == SELECTOR_CAL_ATTEMPTS) {
          // This is the fifth try, get help from the user.


          // reset the attempt counter, then....
          attempt=0;
        }
        // try again
        fail_out = 1;
        break;
      }

      tries[i] = steps;
      left_flex += steps;
    }
    if(fail_out) continue; // propagate fail from inner loop

    // are all our tries within ~.1mm?
    left_flex /= SELECTOR_CAL_SAMPLES;
    fail_out = 0;
    for(i=1; i<SELECTOR_CAL_SAMPLES; i++){
      steps = left_flex - tries[i];
      if (steps < -SELECTOR_CAL_TOLERANCE || steps > SELECTOR_CAL_TOLERANCE){
        // out of bounds.  fail.
        if(attempt == SELECTOR_CAL_ATTEMPTS) {
          // this is the fifth try, get help from the user.


          // reset the attempt counter, then....
          attempt=0;
        }
        // try again
        fail_out = 1;
        break;
      }
    }
    if(fail_out) continue; // propagate fail from inner loop

    left_flex -= SELECTOR_CAL_BACKOFF_STEPS;

    // Success!  Back off current position by the left flex amount.
    selector_cal_guard_move(left_flex);
    tmc2130_init(tmc2130_mode);

    // based on our measurements... which version of the MMU2S or +X is this?
    // cheat for now
    extruders = 6;

    // save what we've found into permanent storage
    SelectorParams::set_extruders(extruders);
    SelectorParams::set_span(raw_span - right_flex - left_flex);
    SelectorParams::set_right_flex(right_flex);
    SelectorParams::set_left_flex(left_flex);
    SelectorParams::set_offset(0);
    selector_park_steps = raw_span - right_flex - left_flex + (selector_steps*(extruders-1));

    return true;
  }
}

// shorter version of the calibration; only need to find the right stop, no side effects
bool home_selector() {
  int i;
  int steps;
  int attempt = 0;

  // if FINDA is sensing filament do not home; indicates filament
  // present, gives user chance to recover by pressing right button
  check_filament_not_present();

  // Now we can attempt calibration.

  tmc2130_init(HOMING_MODE);   // Same as normal mode right now, might not always be

  // try five times before asking for help
  while(1){
    int32_t raw_span = 0;
    int32_t right_flex = 0;
    int32_t left_flex = 0;
    attempt++;

    // Go directly to right stop
    steps = selector_cal_guard_move(SELECTOR_CAL_LEADSCREW_STEPS);
    if (steps == SELECTOR_CAL_LEADSCREW_STEPS) {
      // Dd not detect right stop.  That's a fail.
      if(attempt == SELECTOR_CAL_ATTEMPTS) {
        // this is the fifth try, get help from the user.


        // reset the attempt counter, then....
        attempt=0;
      }
      // try again
      continue;
    }

    // verify selector is really at the right stop and not just stuck
    // or stuttering.
    bool fail_out = 0;
    for(i=0; i < SELECTOR_CAL_SAMPLES; i++){
      steps = selector_cal_guard_move(-SELECTOR_CAL_BACKOFF_STEPS);
      if(steps < SELECTOR_CAL_BACKOFF_STEPS){
        // Jammed? We shouldn't have a short count. Immediately fail and ask for help.



        // then reset attempt counter and continue
        attempt = 0;
        fail_out = 1;
        break;
      }

      // Hit the right stop again and note the number of steps.
      steps = selector_cal_guard_move(SELECTOR_CAL_BACKOFF_STEPS*2);
      if(steps == SELECTOR_CAL_BACKOFF_STEPS*2){
        // did not detect stop.  That's a fail.
        if(attempt == SELECTOR_CAL_ATTEMPTS) {
          // This is the fifth try, get help from the user.


          // reset the attempt counter, then....
          attempt=0;
        }
        // try again
        fail_out = 1;
        break;
      }
    }
    if(fail_out) continue; // propagate fail from inner loop


    // Done.  Move to position 0
    int span = SelectorParams::get_span() - SelectorParams::get_offset();
    int flex = SelectorParams::get_right_flex();
    selector_park_steps = span - (selector_steps*(extruders-1));

    tmc2130_init(tmc2130_mode);
    move_proportional(0,flex-span);
    return true;
  }
}


//! @brief Home both idler and selector if already not done
void calibrate()
{
    home_idler();
    calibrate_selector();
}


//! @brief Home both idler and selector if already not done
void home()
{
    home_idler();
    home_selector();
    shr16_set_led(0x155);
    shr16_set_led(0x000);
    set_extruder_led(active_extruder, GREEN);
}

void move_proportional(int _idler, int _selector)
{
	// get steps to be done and set direction
	_idler = set_idler_direction(_idler);
	_selector = set_selector_direction(_selector);

	float _idler_step = _selector ? (float)_idler/(float)_selector : 1.0;
	float _idler_pos = 0;
	int delay = 2500; //microstep period in microseconds
	const int _start = _selector - 250;
	const int _end = 250;

	while (_selector != 0 || _idler != 0 )
	{
		if (_idler_pos >= 1)
		{
			if (_idler > 0) { idler_step_pin_set(); }
		}
		if (_selector > 0) { selector_step_pin_set(); }

		delayMicroseconds(delay>>1);

		if (_idler_pos >= 1)
		{
			if (_idler > 0) { idler_step_pin_reset(); _idler--;  }
		}

		if (_selector > 0) { selector_step_pin_reset(); _selector--; }
		delayMicroseconds(delay>>1);

		if (_idler_pos >= 1)
		{
			_idler_pos = _idler_pos - 1;
		}


		_idler_pos = _idler_pos + _idler_step;

		if (delay > 900 && _selector > _start) { delay -= 10; }
		if (delay < 2500 && _selector < _end) { delay += 10; }

	}
}

void set_idler_dir_down()
{
	shr16_set_dir(shr16_get_dir() & ~4);
	//shr16_set_dir(shr16_get_dir() | 4);
}
void set_idler_dir_up()
{
	shr16_set_dir(shr16_get_dir() | 4);
	//shr16_set_dir(shr16_get_dir() & ~4);
}


int set_idler_direction(int _steps)
{
	if (_steps < 0)
	{
		_steps = _steps * -1;
		set_idler_dir_down();
	}
	else
	{
		set_idler_dir_up();
	}
	return _steps;
}
int set_selector_direction(int _steps)
{
	if (_steps < 0)
	{
		_steps = _steps * -1;
		shr16_set_dir(shr16_get_dir() & ~2);
	}
	else
	{
		shr16_set_dir(shr16_get_dir() | 2);
	}
	return _steps;
}
void set_pulley_dir_push()
{
	shr16_set_dir(shr16_get_dir() & ~1);
}
void set_pulley_dir_pull()
{
	shr16_set_dir(shr16_get_dir() | 1);
}

//! @brief Park idler
//! each filament selected has its park position, there is no park position for all filaments.
//! @param _unpark
//!  * false park
//!  * true engage
void park_idler(bool _unpark)
{
    if (_unpark && isIdlerParked) // get idler in contact with filament
    {
        move_proportional(idler_parking_steps, 0);
        isIdlerParked = false;
    }
    else if (!_unpark && !isIdlerParked) // park idler so filament can move freely
    {
        move_proportional(idler_parking_steps*-1, 0);
        isIdlerParked = true;
    }
}
