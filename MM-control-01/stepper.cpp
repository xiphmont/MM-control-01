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

// This file maintains the physical reality of stepper state and position

int8_t filament_type[MAX_EXTRUDERS] = {-1, -1, -1, -1, -1, -1, -1, -1};
static bool isIdlerParked = false;
static bool isIdlerHomed = false;
static int  ActiveIdler = -1;
static int idler_offset;

// 40 degrees of rotation between idlers
static const int idler_steps = 355;
// park position is ~ 24.4 degrees ahead of engaged
static const int idler_parking_steps = 217;

static bool isSelectorHomed = false;
static int  ActiveSelector = -1;
static int  selector_left_offset = 0;
static int  selector_right_offset = 0;
static int  selector_span = 0;

// 14mm of leadscrew travel, minus PETG shrinkage
static const int selector_steps = 697;
static bool s_has_door_sensor = false;

static void move_proportional(int _next_idler, bool _park, int _next_selector, uint8_t _tries);
static int set_idler_direction(int _steps);
static int set_selector_direction(int _steps);

void stepper_state_init(){
  tmc2130_init(HOMING_MODE);
  tmc2130_read_gstat(); //consume reset after power up

  // we may have need of a few cached values before homing
  extruders = SelectorParams::get_extruders();
  idler_offset = SelectorParams::get_idler_offset();
  selector_left_offset = SelectorParams::get_left_offset();
}

static int selector_park_steps(){
  return selector_span - selector_left_offset - (selector_steps*(extruders-1));
}

//! @brief Compute steps for selector needed to change filament
//! @param current_filament Currently selected filament
//! @param next_filament Filament to be selected
//! @return selector steps
static int get_selector_steps(int next_filament){
  int current_filament = ActiveSelector;
  int steps = 0;
  if (current_filament == -1) { // coming off left stop; right now we never do this
    steps += selector_left_offset;
    current_filament++;
  }
  if (current_filament == extruders+1) { // coming off right stop
    steps -= selector_right_offset;
    current_filament--;
  }
  if (current_filament == extruders) { // coming out of park
    steps -= selector_park_steps();
    current_filament--;
  }
  if (next_filament == extruders) { // going into park
    steps += selector_park_steps();
    next_filament--;
  }
  steps += (next_filament - current_filament) * selector_steps;
  return steps;
}

//! @brief Compute steps for idler needed to change filament
//! @param current_filament Currently selected filament
//! @param next_filament Filament to be selected
//! @return idler steps
static int get_idler_steps(int next_filament, bool park) {
  int current_filament = ActiveIdler;
  int steps = 0;
  if (current_filament == -1) { // coming off stop
    steps -= idler_offset;
    current_filament = 0;
  }
  if(isIdlerParked){
    steps += idler_parking_steps;
  }
  if(park){
    steps -= idler_parking_steps;
  }
  steps += (current_filament - next_filament) * idler_steps;
  return steps;
}

void do_pulley_step() {
    pulley_step_pin_set();
    asm("nop");
    pulley_step_pin_reset();
    asm("nop");
}

// No idea how to fix something?  Wail plaintively until the user
// intervenes and then presses a button to continue.
static void cry_for_help(){
  while(1) {
    signal_load_failure();
    if (buttonPressed() == Btn::middle) break;
  }
  set_extruder_led(active_extruder, GREEN);
}

//#define LED_SG_DIAG 1
//#define DUMP_STEPPER_CAL 1

// back-emf sense can have substantial step-to-step ripple.  Model
// this as a DC-offset per full step mod 4.  The filter below uses four
// lowpasses to approximate the DC-offset of each phase and remove the
// offset ripple from StallGuard readings.  Imbalance rejection is
// about -48dB without affecting StallGuard reaction speed.
typedef struct {
  uint8_t bootstrap; // progressive setup + fast start
  uint8_t micromask;
  uint8_t microstep;
  uint8_t phase;
  int16_t lowpass[4];
  int16_t hold;
  uint16_t last;
#ifdef DUMP_STEPPER_CAL
  uint16_t count;
#endif
} four_phase;

static bool dump = false;
void four_phase_init(four_phase *f, uint8_t micro_log){
  f->micromask = ((int)1 << micro_log) - 1;
  f->bootstrap = 1;
  f->microstep = 0;
  f->phase = 3; // latch first input value, then ignore next four
  f->lowpass[0] = 0;
  f->lowpass[1] = 0;
  f->lowpass[2] = 0;
  f->lowpass[3] = 0;
  f->hold = 0;
  f->last = 0x7fff;
#ifdef DUMP_STEPPER_CAL
  f->count = 0;
#endif
}

// filter ignores initial value plus next four full step transitions
// to let SG initialize.  It then needs four full steps to minimally
// prime the lowpasses, 32 steps to reach steady-state
int four_phase_filter(four_phase *f, uint16_t val){
  f->microstep += 1;
  f->microstep &= f->micromask;
  // SG updates are not aligned to beginning of travel (we could check
  // the microstep register, but for now zero-knowledge priming works
  // best), so freewheel at max update period but lock phase at any
  // change
  if(!f->microstep || val != f->last){
    f->phase += 1;
    f->phase &= 0x3;
    f->microstep = 0;
    if(f->bootstrap < 4) f->lowpass[f->phase] = val;
    f->hold = f->lowpass[f->phase] - val;
    f->lowpass[f->phase] = (f->bootstrap*f->hold+(val<<3))>>3;
    f->last = val;
    if(!f->phase && f->bootstrap != 7) f->bootstrap++;
#ifdef DUMP_STEPPER_CAL
    if(dump)fprintf_P(uart_com, PSTR("%d %d %dlog\n"),f->count,val,f->hold);
#endif
  }
#ifdef DUMP_STEPPER_CAL
  f->count++;
#endif
  return f->hold;
}

// fixed-rate move for idler calibration that watches StallGuard and
// stops if we see a marked change in motor load. Returns the number
// of microsteps actually executed (always positive)
static int idler_cal_guard_move(int microsteps) {
  int i;
#if LED_SG_DIAG // watch stall guard load signal on LEDs in realtime; useful for tuning
  int min=5;
  int max=0;
#endif
  four_phase sg_filter;
  four_phase_init(&sg_filter, 4); // 16 microsteps-per-step

  // interesting startup bug; on first movement, the SG values are
  // often scaled... wrong?  Somehow, toggling the direction pin
  // clears the problem.
  set_idler_direction(-microsteps);
  microsteps = set_idler_direction(microsteps);
  // Perform steps.  DO NO UNNEEDED WORK.  The original code caused
  // the Triaminic to stutter occasionally (not certain of cause---
  // jitter, shr16 hazards, reconfiguring all drivers to same settings
  // repeatedly, all/none/other?) A stutter did not a lose a step, but
  // when it happened we got a stall notification, same as hitting an
  // end stop.
  for (i = 0; i < microsteps; i++) {
    idler_step_pin_set();
    delayMicroseconds(IDLER_CAL_DELAY/2);
    idler_step_pin_reset();
    delayMicroseconds(IDLER_CAL_DELAY/2);
    // Read StallGuard register.  Keep the read in the timing
    // loop even if we're going to disregard it due to startup.
    int sg = tmc2130_read_sg(AX_IDL);
    int sgf = four_phase_filter(&sg_filter, sg);
    if (i>IDLER_CAL_STALLGUARD_PRESTEPS){
      if(sgf >= 350 || sg==0) break;
    }
#if LED_SG_DIAG
    {
      // diagnostic to display useful realtime stallguard data on
      // LEDs. DEBUGGING ONLY, this is potentially hazardous to timing.
      uint16_t led=0;
      int j;
      sg = (sg >> 5) | (sg?1:0);
      for(j=0;j<5;j++)
        led |= (sg>>j?GREEN<<2*(5-j):0);
      if(i>IDLER_CAL_STALLGUARD_PRESTEPS){
        if(min>sg) min = sg;
        if(min<0) min=0;
        if(max<sg) max = sg;
        if(max>5)max=5;
        led |= ORANGE << 2*(5-min);
        led |= GREEN << 2*(5-max);
        shr16_set_led(led);
      }
    }
#endif
  }
  return i;
}

//! @brief calibrate idler to stop, update idler calibration state
//! (but not permanent storage state, that's done in calibrate() to
//! save wear on EEPROM)

//! This is a fast enough process we use it for homing as well.
//!
//! @retval 0 Succeeded
//! @retval nonzero Error code
static int calibrate_idler(){
  int i;
  int attempt = 0;

  tmc2130_init(HOMING_MODE);

  // try the calibration process five times before asking for help
  while(1){
    attempt++;

    // how many times have we already tried and failed?  Indicate progress outside stepper loop.
    uint16_t led = 0;
    for (i = 0; i < attempt; i++) led |= ORANGE << 2*(5-i);
    shr16_set_led(led);

    // if we're already at the stop, we'll hit it immediately before
    // stallguard can see it.  That will bounce the idler and the
    // repeatability test will fail.  Begin by backing away even
    // though we don't know where we are.  Worst case, it makes a
    // little noise.
    idler_cal_guard_move(-IDLER_CAL_BACKOFF_STEPS*2);

    // now go to stop
    int ret = idler_cal_guard_move(IDLER_CAL_BARREL_STEPS);
    if (ret == IDLER_CAL_BARREL_STEPS) {
      // Did not detect stop.  That's a fail.
      if(attempt == IDLER_CAL_ATTEMPTS) return ERR_IDLER_CAL_MISSING_STOP;
      // try again
      continue;
    }
    // verify idler is really at the stop and not just stuck or
    // stuttering.  Hit the stop a few times from the same short
    // distance and see if we get a repeatable number of steps.
    int tries[IDLER_CAL_SAMPLES];
    bool fail_out = 0;
    int bounce = 0;
    for(i=0; i < IDLER_CAL_SAMPLES; i++){
      ret = idler_cal_guard_move(-IDLER_CAL_BACKOFF_STEPS);
      if(ret < IDLER_CAL_BACKOFF_STEPS) return ERR_IDLER_CAL_JAM;

      // Hit the stop again and note the number of steps.
      ret = idler_cal_guard_move(IDLER_CAL_BACKOFF_STEPS*2);
      if(ret == IDLER_CAL_BACKOFF_STEPS*2){
        // did not detect stop.  That's a fail.
        if(attempt == IDLER_CAL_ATTEMPTS) return ERR_IDLER_CAL_MISSING_STOP;
        // try again
        fail_out = 1;
        break;
      }
      tries[i] = ret;
      bounce += ret;
    }
    if(fail_out) continue; // propagate fail from inner loop

    // are all our tries within tolerance?
    bounce = (bounce + (IDLER_CAL_SAMPLES>>1)) / IDLER_CAL_SAMPLES;  // now average travel
    fail_out = 0;
    for(i=0; i<IDLER_CAL_SAMPLES; i++){
      ret = bounce - tries[i];
      if (ret < -IDLER_CAL_TOLERANCE || ret > IDLER_CAL_TOLERANCE){
        // out of bounds tolerance.
        if(attempt == IDLER_CAL_ATTEMPTS) return ERR_IDLER_CAL_TOLERANCE;
        // try again
        fail_out = 1;
        break;
      }
    }
    if(fail_out) continue; // propagate fail from inner loop
    bounce -= IDLER_CAL_BACKOFF_STEPS; // now bounce and not travel

    // SUCCESS! We've passed the stop repeatability test.
    // Leave our position here at the stop.  We may have a synchronized move afterward.
    isIdlerHomed = true;
    isIdlerParked = false;
    ActiveIdler = -1;
    idler_offset = IDLER_CAL_HOMING_STEPS;
    tmc2130_init(tmc2130_mode);
    return 0;
  }
}

// forces immediate home operation
int home_idler(void){
  int ret = calibrate_idler();
  if(ret) return ret;
  idler_offset = SelectorParams::get_idler_offset();
  return 0;
}

// marks idler in need of homing
void reset_idler(){
  isIdlerHomed = false;
}

// fixed-rate move for selector calibration that watches StallGuard and
// stops if we see a marked change in motor load. Returns the number
// of microsteps actually executed (always positive)
static int selector_cal_guard_move(int microsteps,
                                    int *flex) {
  int i;
  *flex = 0;
#if LED_SG_DIAG // watch stall guard load signal on LEDs in realtime; useful for tuning
  int min=5;
  int max=0;
#endif
  four_phase sg_filter;
  four_phase_init(&sg_filter, 1); // 2 microsteps-per-step

  // interesting startup bug; on first movement, the SG values are scaled... wrong?
  // Somehow, toggling the direction pin clears the problem.
  set_selector_direction(-microsteps);
  microsteps = set_selector_direction(microsteps);
  while(digitalRead(A1) == 1 && microsteps) {
    // something went very wrong; we can't move the selector with filament loaded.
    cry_for_help();
  }
  // Perform steps.  DO NO UNNEEDED WORK. The original code caused the
  // Triaminic to stutter occasionally (not certain of cause---
  // jitter, shr16 hazards, reconfiguring all drivers to same settings
  // repeatedly, all/none/other?) A stutter did not a lose a step, but
  // when it happened we got a stall notification, same as hitting an
  // end stop.
  for (i = 0; i < microsteps; i++) {
    selector_step_pin_set();
    delayMicroseconds(SELECTOR_CAL_DELAY/2);
    selector_step_pin_reset();
    delayMicroseconds(SELECTOR_CAL_DELAY/2);
    // Read StallGuard register.  Keep the read in the timing
    // loop even if we're going to disregard it due to startup.
    int sg = tmc2130_read_sg(AX_SEL);
    int sgf = four_phase_filter(&sg_filter,sg);
    if (i>SELECTOR_CAL_STALLGUARD_PRESTEPS){
      if(sgf >= 50) {
        (*flex)++;
      }else{
        (*flex)=0;
      }
      if(sgf >= 250 || sg==0) break;
    }
#if LED_SG_DIAG // diagnostic to display useful realtime stallguard
                // data on LEDs. DEBUGGING ONLY, this is potentially
                // hazardous to timing.
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
#ifdef DUMP_STEPPER_CAL
  if (dump) {
    fprintf_P(uart_com, PSTR("FLEXlog\n"));
    fprintf_P(uart_com, PSTR("%dlog\n"),*flex);
  }
#endif
  return i;
}

// Hit the stop a few times and see if our stall distances are
// repeatable.  Assumes we begin at the stop.
int selector_repeat_stop(int steps, int dir, int *deviation, int *bounce, int *flex){
  int i;
  int tries[SELECTOR_CAL_SAMPLES];
  int max = 0;
  int ret;
  *flex = 0;
  *bounce = 0;
  for(i=0; i < SELECTOR_CAL_SAMPLES; i++){
    // back off the stop
    tries[i] = selector_cal_guard_move(-dir*steps, &ret);
    if(tries[i] < steps){
      // Jammed? We shouldn't have a short count here ever.
      return ERR_SELECTOR_JAM;
    }

    // Hit the stop again and note the number of steps.
    tries[i] = selector_cal_guard_move(dir*steps*2, &ret);
    if(tries[i] == steps*2){
      // did not detect stop.
      return ERR_SELECTOR_MISSING_STOP;
    }
    *bounce += tries[i];
    *flex += ret;
  }
  *bounce = (*bounce + (SELECTOR_CAL_SAMPLES>>1)) / SELECTOR_CAL_SAMPLES;
  *flex = (*flex + (SELECTOR_CAL_SAMPLES>>1)) / SELECTOR_CAL_SAMPLES;

  // Bounce and flex are different ways of measuring unusable slop
  // distance at the stop.  Flex measures from the inflection point of
  // the load curve to the stopping point.  Bounce is how many steps
  // are lost due to exceeding dynamic holding current capacity upon
  // travel reversal.  Flex and bounce typically agree fairly closely
  // when stallguard is properly tuned for our purposes.  Flex is
  // usually higher on the left stop where the plastic flexes with
  // less force over a longer distance, and bounce may be higher on
  // the right where the stop is more rigid.  Use the max of the two
  // to determine how much travel to remove from the usable span.

  for(i=0; i<SELECTOR_CAL_SAMPLES; i++){
    int diff = abs(*bounce - tries[i]);
    if (diff > max) max = diff;
  }
  *deviation = max;
  *bounce -= steps;
  return 0;
}

// Side effects galore!  This determines positions of endstops, how
// much flex we're dealing with, intuits number of extruders from
// that, and then saves it all to permanent storage.
static int calibrate_selector() {
  int i;
  int attempt = 0;

  // if FINDA is sensing filament do not home; indicates filament
  // present, gives user chance to recover by pressing right button.
  // Higher-level code should never call this if FINDA
  // shows filament, but doublecheck.
  check_filament_not_present();

  // Now we can attempt calibration.
  tmc2130_init(HOMING_MODE);

  // full calibration: left, right, and span try the entire process
  // five times before asking for help, unless it's an 'impossible'
  // error.
  while(1){
    int raw_span = 0;
    int left_flex = 0;
    int left_bounce = 0;
    int right_flex = 0;
    int right_bounce = 0;
    int deviation = 0;
    attempt++;

    // how many times have we already tried and failed?  Indicate progress outside stepper loop.
    uint16_t led = 0;
    for (i = 0; i < attempt; i++) led |= ORANGE << 2*(5-i);
    shr16_set_led(led);

    // If we're at the left stop already, we'll hit it immediately
    // before stallguard can see it.  That will bounce the selector
    // unpredictably and the repeatability test will fail.  Begin by
    // backing away even though we don't know where we are.  If we hit
    // the right stop, that's noisy but OK.
    selector_cal_guard_move(SELECTOR_CAL_BACKOFF_STEPS, &left_flex);

    // Now go directly to left stop
    int ret = selector_cal_guard_move(-SELECTOR_CAL_LEADSCREW_STEPS, &left_flex);
    if (ret == SELECTOR_CAL_LEADSCREW_STEPS) {
      // Did not detect stop.  Well.  That's a bad start.
      if(attempt == SELECTOR_CAL_ATTEMPTS) return ERR_SELECTOR_MISSING_STOP;
      // try again
      continue;
    }

    // verify selector is really at the left stop and not just stuck
    ret = selector_repeat_stop(SELECTOR_CAL_BACKOFF_STEPS, -1,
                               &deviation, &left_bounce, &left_flex);
    if (ret == ERR_SELECTOR_JAM) return ret; // don't retry,  we'll just jam again.
    if (ret) {
      if(attempt == SELECTOR_CAL_ATTEMPTS) return ret;
      continue;
    }
#ifdef DUMP_STEPPER_CAL
    // curious about offset.....
    if(dump){
      fprintf_P(uart_com, PSTR("LEFTlog\n"));
      fprintf_P(uart_com, PSTR("BOUNCElog\n"));
      fprintf_P(uart_com, PSTR("%dlog\n"),left_bounce);
      fprintf_P(uart_com, PSTR("DEVlog\n"));
      fprintf_P(uart_com, PSTR("%dlog\n"),deviation);
      fprintf_P(uart_com, PSTR("FLEXlog\n"));
      fprintf_P(uart_com, PSTR("%dlog\n"),left_flex);
    }
#endif
    if (deviation > SELECTOR_CAL_TOLERANCE){
      if(attempt == SELECTOR_CAL_ATTEMPTS) return ERR_SELECTOR_LEFT_TOLERANCE;
      continue;
    }

    // We've passed the left stop checks.
    // Currently at hard left stop.  Now find right stop.
    ret = selector_cal_guard_move(SELECTOR_CAL_LEADSCREW_STEPS, &right_flex);
    if (ret == SELECTOR_CAL_LEADSCREW_STEPS) {
      // Did not detect stop.
      if(attempt == SELECTOR_CAL_ATTEMPTS) return ERR_SELECTOR_MISSING_STOP;
      // try again
      continue;
    }

    // We provisionally have our full span measurement
    raw_span = ret;

    // verify selector is really at right stop and measure repeatability
    ret = selector_repeat_stop(SELECTOR_CAL_BACKOFF_STEPS, 1,
                               &deviation, &right_bounce, &right_flex);
    if (ret == ERR_SELECTOR_JAM) return ret; // don't retry,  we'll just jam again.
    if (ret) {
      if(attempt == SELECTOR_CAL_ATTEMPTS) return ret;
      continue;
    }
#ifdef DUMP_STEPPER_CAL
    // curious about offset.....
    if (dump){
      fprintf_P(uart_com, PSTR("RIGHTlog\n"));
      fprintf_P(uart_com, PSTR("BOUNCElog\n"));
      fprintf_P(uart_com, PSTR("%dlog\n"),right_bounce);
      fprintf_P(uart_com, PSTR("DEVlog\n"));
      fprintf_P(uart_com, PSTR("%dlog\n"),deviation);
      fprintf_P(uart_com, PSTR("FLEXlog\n"));
      fprintf_P(uart_com, PSTR("%dlog\n"),right_flex);
      delay(100); // don't send OK until we can flush log.  Cheap race fix.
    }
#endif
    if (deviation > SELECTOR_CAL_TOLERANCE){
      if(attempt == SELECTOR_CAL_ATTEMPTS){
        return ERR_SELECTOR_RIGHT_TOLERANCE;
      }
      continue;
    }
    // We've passed the right stop checks.

    // Selector is currently located at the bounce position off the
    // right stop.  Leave our position where it is as we may have a
    // coordinated move immediately following calibration

    // based on the raw span measurement... which version of the MMU2S/X is this?
    {
      int first_filament = 0;
      if (raw_span >= 3900 && raw_span < 4100){
        // MMU2S+6: all alignments based off the right stop
        // ideal span is 3983 steps with filament 0 at -3933 steps from right full stop
        extruders = 6;
        first_filament = 3933;
      } else if (raw_span > 3688 && raw_span < 3900){
        // Stock MMU2S: all alignments based off the right stop
        // ideal span is 3758 steps with filament 0 at -3708 steps from right full stop
        extruders = 5;
        first_filament = 3708;
      } else {
        // we don't do any others yet
      }

      // Selector span represents the usable range.  Don't let the
      // selector get into the flex/bounce on either side even if user
      // requests it via offset adjustment.  Left bounce (but not any
      // additional flex; flex and bounce overlap on both sides) is
      // already absent from the raw span measurement. Right
      // bounce/flex is still included.  The 8 below is four full
      // steps; it's a minimal flex margin guard.
      selector_span = raw_span -
        max(left_flex - left_bounce, 8) -
        max(right_flex, right_bounce);
      // right offset: distance from where we are now (bounced off
      // right stop) to usable span.  This distance exists *outside*
      // the usable span.
      selector_right_offset = max(right_flex - right_bounce, 0);
      // left offset: distance from left side of usable range to first
      // filament.  This distance exists *inside* the usable span.
      selector_left_offset = selector_span - first_filament;
    }

    if(selector_left_offset<0  ||
       selector_left_offset>255 ||
       selector_right_offset>255 ||
       selector_park_steps() < 100){
      if(attempt == SELECTOR_CAL_ATTEMPTS) return ERR_SELECTOR_DIMENSIONS_FAILED;
      continue;
    }

    // save what we've found into permanent storage
    SelectorParams::set_extruders(extruders);
    SelectorParams::set_span(selector_span);
    SelectorParams::set_right_offset(selector_right_offset);
    SelectorParams::set_left_offset(selector_left_offset);
    isSelectorHomed = true;
    ActiveSelector = extruders+1; // at the right stop
    tmc2130_init(tmc2130_mode);
    return 0;
  }
}

// shorter version of the calibration; only need to find the right stop.
int home_selector() {
  int dummy;
  int attempt = 0;

  // if FINDA is sensing filament do not home; indicates filament
  // present, gives user chance to recover by pressing right button
  check_filament_not_present();

  tmc2130_init(HOMING_MODE);
  // try five times before asking for help
  while(1){
    attempt++;

    selector_cal_guard_move(-SELECTOR_CAL_BACKOFF_STEPS, &dummy);

    // Go directly to right stop
    int ret = selector_cal_guard_move(SELECTOR_CAL_LEADSCREW_STEPS, &dummy);
    if (ret == SELECTOR_CAL_LEADSCREW_STEPS) {
      // Did not detect stop.
      if(attempt == SELECTOR_CAL_ATTEMPTS) return ERR_SELECTOR_MISSING_STOP;
      // try again
      continue;
    }

    // verify selector is really at the right stop and not just stuck
    // or stuttering.
    ret = selector_repeat_stop(SELECTOR_CAL_BACKOFF_STEPS, 1, &dummy, &dummy, &dummy);
    if (ret == ERR_SELECTOR_JAM) return ret; // don't retry,  we'll just jam again.
    if (ret) {
      if(attempt == SELECTOR_CAL_ATTEMPTS) return ret;
      continue;
    }

    // Success!  Leave position at right stop, update state.
    selector_span = SelectorParams::get_span();
    selector_right_offset = SelectorParams::get_right_offset();
    selector_left_offset = SelectorParams::get_left_offset();
    isSelectorHomed = true;
    ActiveSelector = extruders+1; // at the right stop

    tmc2130_init(tmc2130_mode);

    return 0;
  }
}

// marks selector in need of homing, but does not perform it immediately
void reset_selector(){
  isSelectorHomed = false;
}

//! @brief Calibrate both idler and selector.
int calibrate(bool retry_forever){
  int ret;
  dump = true;
  while(1){
    if((ret=calibrate_idler())){
      if (retry_forever) continue;
      dump = false;
      return ret;
    }
    break;
  }
  while(1){
    if((ret=calibrate_selector())){
      if(retry_forever) continue;
      dump = false;
      return ret;
    }
    break;
  }
      dump = false;
  SelectorParams::set_idler_offset(idler_offset);
  shr16_set_led(0x155);
  shr16_set_led(0x000);
  set_extruder_led(active_extruder, GREEN);
  return 0;
}

//! @brief Home both idler and selector.
int home(bool retry_forever){
  int ret;
  while(1){
    if((ret=home_idler())){
      if (retry_forever) continue;
      return ret;
    }
    break;
  }
  while(1){
    if((ret=home_selector())){
      if(retry_forever) continue;
      return ret;
    }
    break;
  }
  return 0;
}

//! @brief returns cached idler offset
uint8_t get_idler_offset(){
  return idler_offset;
}

//! @brief returns cached selector left offset
uint8_t get_selector_offset(){
  return selector_left_offset;
}

//! sets new absolute idler offset
bool set_idler_offset(uint8_t new_offset){
  if(!isIdlerHomed){
    if(home_idler())
      return false;
  }

  // update the actual position in realtime
  // guard move used for convenience, not to guard.  Offset can't get past the stop.
  if(new_offset != idler_offset){
    idler_cal_guard_move(idler_offset-new_offset);
    idler_offset = new_offset;
    SelectorParams::set_idler_offset(idler_offset);
  }
  return true;
}

//! @brief sets new selector left offset; if done in steps of 8, stall guarding works
bool set_selector_offset(uint8_t new_offset){
  if(digitalRead(A1)) return false;
  if(!isSelectorHomed){
    if(home_selector())
      return false;
  }

  // Move selector to active filament.  If we're at service position go to last filament.
  move_proportional(ActiveIdler, isIdlerParked,
                    ActiveSelector < extruders ? ActiveSelector : extruders - 1, 1);

  // update the actual position in realtime
  if(new_offset != selector_left_offset) {
    int steps = new_offset-selector_left_offset;
    int dummy;
    selector_cal_guard_move(steps, &dummy);
    selector_left_offset += steps;
    SelectorParams::set_left_offset(selector_left_offset);
  }
  return true;
}

//! @brief move idler and selector in concert (becuase it looks cool)
//! to desired location. In case of drive error, reset, re-home and
//! try to recover 3 times.  If the drive error is permanent call
//! unrecoverable_error();
static void move_proportional(int _next_idler, bool _park, int _next_selector, uint8_t _tries){
  while (_tries--) {

    int idler_steps = 0;
    int selector_steps = 0;

    while(digitalRead(A1) == 1 && selector_steps) {
      // something went very wrong; we can't move the selector with filament loaded.
        cry_for_help();
    }

    // check homing status, then get steps to be done and set direction
    if(ActiveIdler != _next_idler || _park != isIdlerParked){
      if(!isIdlerHomed){
        if (home_idler())
          continue;
      }
      if(_next_idler == -1) _next_idler = 0;
      idler_steps = get_idler_steps(_next_idler, _park);
      idler_steps = set_idler_direction(idler_steps);
      isIdlerParked = _park;
      ActiveIdler = _next_idler;
    }
    if(ActiveSelector != _next_selector){
      if(!isSelectorHomed){
        if (home_selector())
          continue;
      }
      selector_steps = get_selector_steps(_next_selector);
      selector_steps = set_selector_direction(selector_steps);
      ActiveSelector = _next_selector;
    }

    float idler_partial = selector_steps ? (float)idler_steps/(float)selector_steps : 1.0;
    float idler_acc = 0;
    int usdelay = 2500; //microstep period in microseconds
    const int start_ramp = selector_steps - 250;
    const int end_ramp = 250;

    while (selector_steps || idler_steps ) {
      if (idler_acc >= 1) {
        if (idler_steps > 0) {
          idler_step_pin_set();
        }
      }
      if (selector_steps > 0) {
        selector_step_pin_set();
      }

      delayMicroseconds(usdelay>>1);

      if (idler_acc >= 1){
        if (idler_steps > 0) {
          idler_step_pin_reset();
          idler_steps--;
        }
      }

      if (selector_steps > 0) {
        selector_step_pin_reset();
        selector_steps--;
      }

      delayMicroseconds(usdelay>>1);

      if (idler_acc >= 1) idler_acc -= 1;
      idler_acc = idler_acc + idler_partial;

      if (usdelay > 900 && selector_steps > start_ramp) usdelay -= 10;
      if (usdelay < 2500 && selector_steps < end_ramp) usdelay += 10;
    }

    if (!tmc2130_read_gstat()) break; // no error, we're done
    else {
      drive_error();
      if (!_tries){
        // lock the doors
        unrecoverable_error();
      }else{
        // try again
        drive_error();
        shr16_set_ena(0);
        delay(10);
        shr16_set_ena(7);
        reset_idler();
        reset_selector();
        tmc2130_init(tmc2130_mode);
      }
    }
  }
}

static int set_idler_direction(int _steps){
  if (_steps < 0) {
    _steps = _steps * -1;
    shr16_set_dir(shr16_get_dir() & ~4);
  } else {
    shr16_set_dir(shr16_get_dir() | 4);
  }
  return _steps;
}

static int set_selector_direction(int _steps){
  if (_steps < 0){
    _steps = _steps * -1;
    shr16_set_dir(shr16_get_dir() & ~2);
  } else {
    shr16_set_dir(shr16_get_dir() | 2);
  }
  return _steps;
}

void set_pulley_dir_push() {
  shr16_set_dir(shr16_get_dir() & ~1);
}

void set_pulley_dir_pull() {
  shr16_set_dir(shr16_get_dir() | 1);
}

//! @brief Park idler
//! each filament selected has its park position, there is no park position for all filaments.
//! @param _unpark
//!  * false park
//!  * true engage
void engage_idler(bool unpark){
  move_proportional(ActiveIdler, !unpark, ActiveSelector, 3);
}

void select_idler(int idler){
  move_proportional(idler, isIdlerParked, ActiveSelector, 1);
}

void select_idler_selector(uint8_t idler, uint8_t selector){
  move_proportional(idler, isIdlerParked, selector, 1);
}

void select_idler_selector(uint8_t idler_selector){
  move_proportional(idler_selector, isIdlerParked, idler_selector, 1);
}

//! @brief unload until FINDA senses end of the filament
static void internal_unload_to_finda()
{
  int delay = 2000; //microstep period in microseconds
  const int _first_point = 1800;

  uint8_t _endstop_hit = 0;

  int _unloadSteps = BowdenLength::get() + 1100;
  const int _second_point = _unloadSteps - 1300;

  set_pulley_dir_pull();

  while (_endstop_hit < 100u && _unloadSteps > 0) {
    do_pulley_step();
    _unloadSteps--;

    if (_unloadSteps < 1400 && delay < 6000) delay += 3;
    if (_unloadSteps < _first_point && delay < 2500) delay += 2;
    if (_unloadSteps < _second_point && _unloadSteps > 5000)
      {
        if (delay > 550) delay -= 1;
        if (delay > 330 && (NORMAL_MODE == tmc2130_mode)) delay -= 1;
      }

    delayMicroseconds(delay);
    if (digitalRead(A1) == 0) _endstop_hit++;
  }
}

void feed_to_bondtech() {
  int stepPeriod = 4500; //microstep period in microseconds
  const uint16_t steps = BowdenLength::get();
  uint8_t tries = 3;
  while(tries--){

    set_pulley_dir_push();
    unsigned long delay = 4500;

    for (uint16_t i = 0; i < steps; i++) {
      delayMicroseconds(delay);
      unsigned long now = micros();

      if (i < 4000){
        if (stepPeriod > 2600) stepPeriod -= 4;
        if (stepPeriod > 1300) stepPeriod -= 2;
        if (stepPeriod > 650) stepPeriod -= 1;
        if (stepPeriod > 350 && (NORMAL_MODE == tmc2130_mode) && s_has_door_sensor) stepPeriod -= 1;
      }
      if (i > (steps - 800) && stepPeriod < 2600) stepPeriod += 10;
      if ('A' == getc(uart_com)) {
        s_has_door_sensor = true;
        tmc2130_disable_axis(AX_PUL, tmc2130_mode);
        engage_idler(false);
        return;
      }
      do_pulley_step();
      delay = stepPeriod - (micros() - now);
    }

    if (!tmc2130_read_gstat()) break;
    else {
      int current_idler = ActiveIdler;
      if (!tries) unrecoverable_error();
      drive_error();
      reset_idler();
      move_proportional(current_idler, false, ActiveSelector, 1); // only move idler, no retries
      internal_unload_to_finda();
    }
  }
}

//! @brief unload to FINDA
//!
//! Check for drive error and try to recover 3 times.
void unload_to_finda(){
  uint8_t tries = 3;
  while(tries--){
    internal_unload_to_finda();
    if (tmc2130_read_gstat() && digitalRead(A1) == 1) {
      int current_idler = ActiveIdler;
      if (!tries) unrecoverable_error();
      drive_error();
      reset_idler();
      move_proportional(current_idler, false, ActiveSelector, 1); // only move idler, no retries
    }else{
      break;
    }
  }
}

void door_sensor_detected(){
    s_has_door_sensor = true;
}
