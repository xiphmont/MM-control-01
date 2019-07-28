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
// steps from last extruder to right stop; get this from calibration
static int selector_park_steps = -1;
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
    steps -= selector_park_steps;
    current_filament--;
  }
  if (next_filament == extruders) { // going into park
    steps += selector_park_steps;
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

// fixed-rate move for idler calibration that watches StallGuard and
// breaks if we see a stall. Returns the number of steps actually
// executed (always positive)
static int idler_cal_guard_move(int steps) {
  int i;
#if LED_SG_DIAG // watch stall guard load signal on LEDs in realtime; useful for tuning
  int min=5;
  int max=0;
#endif
  steps = set_idler_direction(steps);
  // Perform steps.  DO NO EXTRA WORK.  No LEDs, no chatting with the
  // other stepper drivers. The original code caused the Triaminic to
  // stutter occasionally (not certain of cause--- jitter, shr16
  // hazards, reconfiguring all drivers to same settings repeatedly,
  // all/none/other?) A stutter did not a lose a step, but when it
  // happened we got a stall notification, same as hitting an end
  // stop.  This is the main reason the original Prusa homing code
  // could be flaky.
  for (i = 0; i < steps; i++) {
    idler_step_pin_set();
    delayMicroseconds(IDLER_CAL_DELAY/2);
    idler_step_pin_reset();
    delayMicroseconds(IDLER_CAL_DELAY/2);
    // Read StallGuard register.  Keep the read in the timing
    // loop even if we're going to disregard it due to startup.
    int sg = tmc2130_read_sg(AX_IDL);
    if (i>IDLER_CAL_STALLGUARD_PRESTEPS && sg==0){
      // we missed a minimum of 4*16 steps to trigger this, remove it from the count
      i-=63;
      if(i<0)i=0;  // JIC
      break;
    }
#if LED_SG_DIAG // diagnostic to display useful realtime stallguard data on LEDs
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
#endif
  }
  return i;
}

//! @brief calibrate idler to stop, update idler calibration state
//! (but not permanent storage state, that's done in calibrate() to
//! save wear on EEPROM

//! This is a fast enough process we use it for homing as well.
//!
//! @retval true Succeeded
//! @retval false Failed
static int calibrate_idler(){
  int i;
  int attempt = 0;

  tmc2130_init(HOMING_MODE);

  // try the calibration process five times before asking for help
  while(1){
    int32_t offset = 0;
    attempt++;

    // how many times have we already tried and failed?  Indicate progress outside stepper loop.
    uint16_t led = 0;
    for (i = 0; i < attempt; i++) led |= ORANGE << 2*(5-i);
    shr16_set_led(led);

    // go to stop
    int ret = idler_cal_guard_move(IDLER_CAL_BARREL_STEPS);
    if (ret == IDLER_CAL_BARREL_STEPS) {
      // Did not detect stop.  That's a fail.
      if(attempt == IDLER_CAL_ATTEMPTS) return ERR_IDLER_CAL_MISSING_STOP;
      // try again
      continue;
    }

    // verify idler is really at the stop and not just stuck
    // or stuttering.

    // Hit the stop a few times from the same short distance and see
    // if we get a repeatable number of steps.
    int tries[IDLER_CAL_SAMPLES];
    bool fail_out = 0;
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
      offset += ret;
    }
    if(fail_out) continue; // propagate fail from inner loop

    // are all our tries within tolerance?
    offset /= IDLER_CAL_SAMPLES;  // now the average
    fail_out = 0;
    for(i=0; i<IDLER_CAL_SAMPLES; i++){
      ret = offset - tries[i];
      if (ret < -IDLER_CAL_TOLERANCE || ret > IDLER_CAL_TOLERANCE){
        // out of bounds tolerance.
        if(attempt == IDLER_CAL_ATTEMPTS) return ERR_IDLER_CAL_TOLERANCE;
        // try again
        fail_out = 1;
        break;
      }
    }
    if(fail_out) continue; // propagate fail from inner loop

    // SUCCESS! We've passed the stop repeatability test. Leave our
    // position at the stop, we may have a synchronized move afterward.
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

// fixed-velocity move for selector calibration that watches StallGuard and
// breaks if we see a stall. Returns the number of steps actually
// executed (always positive).
static int selector_cal_guard_move(int steps, int step_half_delay) {
  int i;
#if LED_SG_DIAG // watch stall guard load signal on LEDs in realtime; useful for tuning
  int j;
  int min=5;
  int max=0;
#endif
  steps = set_selector_direction(steps);
  while(digitalRead(A1) == 1 && steps) {
    // something went very wrong; we can't move the selector with filament loaded.
    cry_for_help();
  }
  // Perform steps.  DO NO EXTRA WORK.  No LEDs, no chatting with the
  // other stepper drivers. The original code caused the Triaminic to
  // stutter occasionally (not certain of cause--- jitter, shr16
  // hazards, reconfiguring all drivers to same settings repeatedly,
  // all/none/other?) A stutter did not a lose a step, but when it
  // happened we got a stall notification, same as hitting an end
  // stop.  This is the main reason the original Prusa homing code
  // could be flaky.
  for (i = 0; i < steps; i++) {
    selector_step_pin_set();
    delayMicroseconds(step_half_delay);
    selector_step_pin_reset();
    delayMicroseconds(step_half_delay);
    // Read StallGuard register.  Keep the read in the timing
    // loop even if we're going to disregard it due to startup.
    uint16_t sg = tmc2130_read_sg(AX_SEL);
    if (i>SELECTOR_CAL_STALLGUARD_PRESTEPS && sg==0){
      // we missed a minimum of 4*2 steps to trigger this, remove it from the count
      i-=7;
      if(i<0)i=0;  // JIC
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

// At normal calibration feed rate, hit the stop a few times and see if our stall
// distances are repeatable.  Assumes we begin at the stop, having
// stalled against it at normal calibration feed rate.
int selector_repeat_stop(int steps, int dir, int *deviation, int *offset){
  int i;
  int tries[SELECTOR_CAL_SAMPLES];
  int average = 0;
  int max = 0;
  for(i=0; i < SELECTOR_CAL_SAMPLES; i++){

    // back off the stop
    int ret = selector_cal_guard_move(-dir*steps, SELECTOR_CAL_DELAY/2);
    if(ret < steps){
      // Jammed? We shouldn't have a short count here ever.
      return ERR_SELECTOR_JAM;
    }

    // Hit the stop again and note the number of steps.
    ret = selector_cal_guard_move(dir*steps*2, SELECTOR_CAL_DELAY/2);
    if(ret == steps*2){
      // did not detect stop.
      return ERR_SELECTOR_MISSING_STOP;
    }
    tries[i] = ret;
    average += ret;
  }
  average =  (average + (SELECTOR_CAL_SAMPLES>>1)) / SELECTOR_CAL_SAMPLES;

  for(i=0; i<SELECTOR_CAL_SAMPLES; i++){
    int diff = abs(average - tries[i]);
    if (diff > max) max=diff;
  }
  *deviation = max;
  *offset = average - steps;
  return 0;
}

// Calibration at normal feedrate has enough speed/inerta to flex the
// stop quite a bit, especially the left stop on a regular MMU2S.

// Hit the stop a few times at a much slower feed rate matching the
// end of the smoothed motion in move_proportional.  The fast
// calibration is used to decide true span and filament location.
// The slow calibration below is used to set the working stops beyond
// which we won't allow even manual menu calibration to stray.

// Assume we've just hit the stop in question at normal speed...
int selector_flex_stop(int steps, int dir, int *deviation, int *offset){
  int i;
  int tries[SELECTOR_CAL_SAMPLES];
  int average = 0;
  int max = 0;
  for(i=0; i < SELECTOR_CAL_SAMPLES; i++){

    // back off the stop (normal speed)
    int ret = selector_cal_guard_move(-dir*steps, SELECTOR_CAL_DELAY/2);
    if(ret < steps){
      // Jammed? We shouldn't have a short count here ever.
      return ERR_SELECTOR_JAM;
    }

    // Hit the stop again *slow*.  We don't count steps here;
    // StallGuard is tightly coupled to velocity matching a given
    // setup and we move too slow to use it.
    tmc2130_init(tmc2130_mode);
    selector_cal_guard_move(dir*steps*2, SELECTOR_CAL_SLOW_DELAY/2);
    tmc2130_init(HOMING_MODE);

    // back off the stop (normal speed)
    ret = selector_cal_guard_move(-dir*steps, SELECTOR_CAL_DELAY/2);
    if(ret < steps){
      // Jammed? We shouldn't have a short count here ever.
      return ERR_SELECTOR_JAM;
    }

    // Hit the stop again at normal speed, and *now* note the number of steps.
    ret = selector_cal_guard_move(dir*steps*2, SELECTOR_CAL_DELAY/2);
    if(ret == steps*2){
      // did not detect stop.
      return ERR_SELECTOR_MISSING_STOP;
    }
    tries[i] = ret;
    average += ret;
  }
  average =  (average + (SELECTOR_CAL_SAMPLES>>1)) / SELECTOR_CAL_SAMPLES;
  for(i=0; i<SELECTOR_CAL_SAMPLES; i++){
    int diff = abs(average - tries[i]);
    if (diff > max) max=diff;
  }
  *deviation = max;
  *offset = average - steps;
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
  // Of course, higher-level code should never call this if FINDA
  // shows filament...
  check_filament_not_present();

  // Now we can attempt calibration.

  // full calibration: left, right, and span try the entire process
  // five times before asking for help, unless it's an 'impossible'
  // error.
  tmc2130_init(HOMING_MODE);
  while(1){
    int32_t raw_span = 0;
    int32_t left_flex = 0;
    int32_t right_flex = 0;
    attempt++;

    // how many times have we already tried and failed?  Indicate progress outside stepper loop.
    uint16_t led = 0;
    for (i = 0; i < attempt; i++) led |= ORANGE << 2*(5-i);
    shr16_set_led(led);

    // Go directly to left stop
    int ret = selector_cal_guard_move(-SELECTOR_CAL_LEADSCREW_STEPS, SELECTOR_CAL_DELAY/2);
    if (ret == SELECTOR_CAL_LEADSCREW_STEPS) {
      // Did not detect stop.  Well.  That's a bad start.
      if(attempt == SELECTOR_CAL_ATTEMPTS) return ERR_SELECTOR_MISSING_STOP;
      // try again
      continue;
    }

    // verify selector is really at the left stop and not just stuck
    // or stuttering.

    // First hit the stop a few times from the same short distance
    // at normal speed and see if we get a repeatable number of steps.
    int dev = 0;
    int fast_off = 0;
    int slow_off = 0;
    ret = selector_repeat_stop(SELECTOR_CAL_BACKOFF_STEPS, -1, &dev, &fast_off);
    if (ret == ERR_SELECTOR_JAM) return ret; // don't retry,  we'll just jam again.
    if (ret) {
      if(attempt == SELECTOR_CAL_ATTEMPTS) return ret;
      continue;
    }
    if (dev > SELECTOR_CAL_TOLERANCE){
      if(attempt == SELECTOR_CAL_ATTEMPTS){
        return ERR_SELECTOR_LEFT_TOLERANCE;
      }
      continue;
    }

    // Verified left stop is repeatable at full speed; now try to determine amount
    // of flex at the left stop.
    ret = selector_flex_stop(SELECTOR_CAL_BACKOFF_STEPS, -1, &dev, &slow_off);
    if (ret == ERR_SELECTOR_JAM) return ret; // don't retry,  we'll just jam again.
    if (ret) {
      if(attempt == SELECTOR_CAL_ATTEMPTS) return ret;
      continue;
    }
    if (dev > SELECTOR_CAL_TOLERANCE){
      if(attempt == SELECTOR_CAL_ATTEMPTS){
        return ERR_SELECTOR_LEFT_FLEX_TOLERANCE;
      }
      continue;
    }

    // We've passed the left stop checks.
    // Flex should be equal to the slow offset minus the fast offset
    left_flex = slow_off - fast_off;
    if (left_flex < 0){
      // shouldn't happen, not necessarily a fail though.  Just assume
      // the fast stop reading is the real one.
      left_flex = 0;
    }

    // Currently at hard left stop from fast feed.  Now find right stop at same speed.
    ret = selector_cal_guard_move(SELECTOR_CAL_LEADSCREW_STEPS, SELECTOR_CAL_DELAY/2);
    if (ret == SELECTOR_CAL_LEADSCREW_STEPS) {
      // Did not detect stop.
      if(attempt == SELECTOR_CAL_ATTEMPTS) return ERR_SELECTOR_MISSING_STOP;
      // try again
      continue;
    }

    // We provisionally have our full span measurement: still need to
    // verify right stop repeatability
    raw_span = ret;

    // verify selector is really at right stop and measure repeatability
    ret = selector_repeat_stop(SELECTOR_CAL_BACKOFF_STEPS, 1, &dev, &fast_off);
    if (ret == ERR_SELECTOR_JAM) return ret; // don't retry,  we'll just jam again.
    if (ret) {
      if(attempt == SELECTOR_CAL_ATTEMPTS) return ret;
      continue;
    }
    if (dev > SELECTOR_CAL_TOLERANCE){
      if(attempt == SELECTOR_CAL_ATTEMPTS){
        return ERR_SELECTOR_RIGHT_TOLERANCE;
      }
      continue;
    }

    // given repeated masurement of right stop, adjust raw span
    raw_span += fast_off;

    // determine right stop flex
    ret = selector_flex_stop(SELECTOR_CAL_BACKOFF_STEPS, 1, &dev, &slow_off);
    if (ret == ERR_SELECTOR_JAM) return ret; // don't retry,  we'll just jam again.
    if (ret) {
      if(attempt == SELECTOR_CAL_ATTEMPTS) return ret;
      continue;
    }
    if (dev > SELECTOR_CAL_TOLERANCE){
      if(attempt == SELECTOR_CAL_ATTEMPTS){
        return ERR_SELECTOR_RIGHT_FLEX_TOLERANCE;
      }
      continue;
    }

    // We've passed the right stop checks.
    right_flex = slow_off - fast_off;
    if(right_flex < 0) {
      // this is somewhat more believable in terms of random chance; the right stop is pretty stiff.
      right_flex = 0;
    }

    // Success!  Leave our position where we are as we may have a
    // coordinated move immediately following calibration
    tmc2130_init(tmc2130_mode);

    // based on our measurements... which version of the MMU2S or +X is this?
    // base our alignment off the right stop
    if(raw_span > 3800){
      if(raw_span < 4100){
        // MMU2S+6: all alignments based off the right stop
        // ideal span is 3883 steps with filament 0 at -3833 steps from right
        extruders = 6;
        selector_span = raw_span - left_flex; // don't allow travel past left_flex offset
        selector_right_offset = right_flex;
        selector_left_offset = selector_span - 3833;
      }else{
        // we don't do anything bigger.... yet.
        // Alert
        return false;
      }
    }else{
      extruders = 5;
      selector_span = raw_span - left_flex; // don't allow travel past left_flex offset
      selector_right_offset = right_flex;
      selector_left_offset = selector_span - 3708;
    }
    selector_park_steps = selector_span - selector_right_offset - selector_left_offset -
      (selector_steps*(extruders-1));

    if(selector_left_offset<0  || selector_left_offset>255) return ERR_SELECTOR_DIMENSIONS_FAILED;
    if(selector_right_offset>255) return ERR_SELECTOR_DIMENSIONS_FAILED;
    if(selector_park_steps<100) return ERR_SELECTOR_DIMENSIONS_FAILED;

    // save what we've found into permanent storage
    SelectorParams::set_extruders(extruders);
    SelectorParams::set_span(selector_span);
    SelectorParams::set_right_offset(selector_right_offset);
    SelectorParams::set_left_offset(selector_left_offset);
    isSelectorHomed = true;
    ActiveSelector = extruders+1; // at the right stop
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

    // Go directly to right stop
    int ret = selector_cal_guard_move(SELECTOR_CAL_LEADSCREW_STEPS, SELECTOR_CAL_DELAY/2);
    if (ret == SELECTOR_CAL_LEADSCREW_STEPS) {
      // Did not detect stop.
      if(attempt == SELECTOR_CAL_ATTEMPTS) return ERR_SELECTOR_MISSING_STOP;
      // try again
      continue;
    }

    // verify selector is really at the right stop and not just stuck
    // or stuttering.
    ret = selector_repeat_stop(SELECTOR_CAL_BACKOFF_STEPS, 1, &dummy, &dummy);
    if (ret == ERR_SELECTOR_JAM) return ret; // don't retry,  we'll just jam again.
    if (ret) {
      if(attempt == SELECTOR_CAL_ATTEMPTS) return ret;
      continue;
    }

    // Success!  Leave position at right stop, update state.
    selector_span = SelectorParams::get_span();
    selector_right_offset = SelectorParams::get_right_offset();
    selector_left_offset = SelectorParams::get_left_offset();
    selector_park_steps = selector_span - selector_right_offset - selector_left_offset -
      (selector_steps*(extruders-1));
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
  while(1){
    if((ret=calibrate_idler())){
      if (retry_forever) continue;
      return ret;
    }
    break;
  }
  while(1){
    if(!(ret=calibrate_selector())){
      if(retry_forever) continue;
      return ret;
    }
    break;
  }
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
    selector_cal_guard_move(steps, SELECTOR_CAL_SLOW_DELAY);
    selector_left_offset += steps;
    selector_park_steps -= steps; // usable span remains the same
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
