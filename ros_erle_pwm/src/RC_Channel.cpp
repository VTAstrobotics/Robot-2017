#include <stdlib.h>
#include <math.h>

#include "RC_Channel.h"

/// global array with pointers to all APM RC channels, will be used by AP_Mount
/// and AP_Camera classes / It points to RC input channels, both APM1 and APM2
/// only have 8 input channels.
RC_Channel *RC_Channel::rc_ch[RC_MAX_CHANNELS];
/*
const AP_Param::GroupInfo RC_Channel::var_info[] PROGMEM = {

    AP_GROUPINFO("MIN",  0, RC_Channel, radio_min, 1100),

    AP_GROUPINFO("TRIM", 1, RC_Channel, radio_trim, 1500),

    AP_GROUPINFO("MAX",  2, RC_Channel, radio_max, 1900),

    AP_GROUPINFO("REV",  3, RC_Channel, _reverse, 1),

    AP_GROUPINFO("DZ",   5, RC_Channel, _dead_zone, 0),

    AP_GROUPEND
};
*/
// setup the control preferences
void
RC_Channel::set_range(int16_t low, int16_t high)
{
    _type           = RC_CHANNEL_TYPE_RANGE;
    _high           = high;
    _low            = low;
    _high_out       = high;
    _low_out        = low;
}

void
RC_Channel::set_range_out(int16_t low, int16_t high)
{
    _high_out       = high;
    _low_out        = low;
}

void
RC_Channel::set_angle(int16_t angle)
{
    _type   = RC_CHANNEL_TYPE_ANGLE;
    _high   = angle;
}

void
RC_Channel::set_default_dead_zone(int16_t dzone)
{
//    if (!_dead_zone.load()) {
//        _dead_zone.set(abs(dzone));
//    }
}

void
RC_Channel::set_reverse(bool reverse)
{
    if (reverse) _reverse = -1;
    else _reverse = 1;
}

bool
RC_Channel::get_reverse(void) const
{
    if (_reverse == -1) {
        return true;
    }
    return false;
}

void
RC_Channel::set_type(uint8_t t)
{
    _type = t;
}

// call after first read
void
RC_Channel::trim()
{
    radio_trim = radio_in;
}

// read input from APM_RC - create a control_in value
void
RC_Channel::set_pwm(int16_t pwm)
{
    radio_in = pwm;

    if (_type == RC_CHANNEL_TYPE_RANGE) {
        control_in = pwm_to_range();
    } else {
        //RC_CHANNEL_TYPE_ANGLE, RC_CHANNEL_TYPE_ANGLE_RAW
        control_in = pwm_to_angle();
    }
}

/*
  call read() and set_pwm() on all channels
 */
void
RC_Channel::set_pwm_all(void)
{
    for (uint8_t i=0; i<RC_MAX_CHANNELS; i++) {
        if (rc_ch[i] != NULL) {
            rc_ch[i]->set_pwm(rc_ch[i]->read());
        }
    }
}

// read input from APM_RC - create a control_in value, but use a 
// zero value for the dead zone. When done this way the control_in
// value can be used as servo_out to give the same output as input
void
RC_Channel::set_pwm_no_deadzone(int16_t pwm)
{
    radio_in = pwm;

    printf("set_pwm_no_deadzone not used!");

    if (_type == RC_CHANNEL_TYPE_RANGE) {
        control_in = pwm_to_range_dz(0);
    } else {
        //RC_CHANNEL_ANGLE, RC_CHANNEL_ANGLE_RAW
        control_in = pwm_to_angle_dz(0);
    }
}

int16_t
RC_Channel::control_mix(float value)
{
    return (1 - abs(control_in / _high)) * value + control_in;
}

// are we below a threshold?
bool
RC_Channel::get_failsafe(void)
{
    return (radio_in < (radio_min - 50));
}

// constrain a int16_t value
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) {
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// returns just the PWM without the offset from radio_min
void
RC_Channel::calc_pwm(void)
{
    if(_type == RC_CHANNEL_TYPE_RANGE) {
        pwm_out         = range_to_pwm();
        radio_out       = (_reverse >= 0) ? (radio_min + pwm_out) : (radio_max - pwm_out);

    }else if(_type == RC_CHANNEL_TYPE_ANGLE_RAW) {
        pwm_out         = (float)servo_out * 0.1f;
        radio_out       = (pwm_out * _reverse) + radio_trim;

    }else{     // RC_CHANNEL_TYPE_ANGLE
        pwm_out         = angle_to_pwm();
        radio_out       = pwm_out + radio_trim;
    }

    radio_out = constrain_int16(radio_out, radio_min, radio_max);
}


/*
  return the center stick position expressed as a control_in value
  used for thr_mid in copter
 */
int16_t
RC_Channel::get_control_mid() const {
    if (_type == RC_CHANNEL_TYPE_RANGE) {
        int16_t r_in = (radio_min+radio_max)/2;

        if (_reverse == -1) {
            r_in = radio_max - (r_in - radio_min);
        }

        int16_t radio_trim_low  = radio_min + _dead_zone;

        return (_low + ((int32_t)(_high - _low) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_max - radio_trim_low));
    } else {
        return 0;
    }
}

// ------------------------------------------

void
RC_Channel::zero_min_max()
{
    radio_min = radio_max = radio_in;
}

void
RC_Channel::update_min_max()
{
//    radio_min = min(radio_min, radio_in);
    if(radio_min> radio_in)
        radio_min = radio_in;

//    radio_max = max(radio_max, radio_in);
    if(radio_max < radio_in)
        radio_max = radio_in;

}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value using the specified dead_zone
 */
int16_t
RC_Channel::pwm_to_angle_dz(uint16_t dead_zone)
{
    int16_t radio_trim_high = radio_trim + dead_zone;
    int16_t radio_trim_low  = radio_trim - dead_zone;

    // prevent div by 0
    if ((radio_trim_low - radio_min) == 0 || (radio_max - radio_trim_high) == 0)
        return 0;

    if(radio_in > radio_trim_high) {
        return _reverse * ((int32_t)_high * (int32_t)(radio_in - radio_trim_high)) / (int32_t)(radio_max  - radio_trim_high);
    }else if(radio_in < radio_trim_low) {
        return _reverse * ((int32_t)_high * (int32_t)(radio_in - radio_trim_low)) / (int32_t)(radio_trim_low - radio_min);
    }else
        return 0;
}

/*
  return an "angle in centidegrees" (normally -4500 to 4500) from
  the current radio_in value
 */
int16_t
RC_Channel::pwm_to_angle()
{
	return pwm_to_angle_dz(_dead_zone);
}


int16_t
RC_Channel::angle_to_pwm()
{
    if((servo_out * _reverse) > 0)
        return _reverse * ((int32_t)servo_out * (int32_t)(radio_max - radio_trim)) / (int32_t)_high;
    else
        return _reverse * ((int32_t)servo_out * (int32_t)(radio_trim - radio_min)) / (int32_t)_high;
}

/*
  convert a pulse width modulation value to a value in the configured
  range, using the specified deadzone
 */
int16_t
RC_Channel::pwm_to_range_dz(uint16_t dead_zone)
{
    int16_t r_in = constrain_int16(radio_in, radio_min, radio_max);

    if (_reverse == -1) {
        r_in = radio_max - (r_in - radio_min);
    }

    int16_t radio_trim_low  = radio_min + dead_zone;

    if (r_in > radio_trim_low)
        return (_low + ((int32_t)(_high - _low) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_max - radio_trim_low));
    else if (dead_zone > 0)
        return 0;
    else
        return _low;
}

/*
  convert a pulse width modulation value to a value in the configured
  range
 */
int16_t
RC_Channel::pwm_to_range()
{
    return pwm_to_range_dz(_dead_zone);
}


int16_t
RC_Channel::range_to_pwm()
{
    if (_high_out == _low_out) {
        return radio_trim;
    }
    return ((int32_t)(servo_out - _low_out) * (int32_t)(radio_max - radio_min)) / (int32_t)(_high_out - _low_out);
}

// ------------------------------------------
float constrain_float(float amt, float low, float high)
{
    // the check for NaN as a float prevents propogation of
    // floating point errors through any function that uses
    // constrain_float(). The normal float semantics already handle -Inf
    // and +Inf
    if (isnan(amt)) {
        return (low+high)*0.5f;
    }
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
float
RC_Channel::norm_input()
{
    float ret;
    if(radio_in < radio_trim)
        ret = _reverse * (float)(radio_in - radio_trim) / (float)(radio_trim - radio_min);
    else
        ret = _reverse * (float)(radio_in - radio_trim) / (float)(radio_max  - radio_trim);
    return constrain_float(ret, -1.0f, 1.0f);
}

float
RC_Channel::norm_input_dz()
{
    int16_t dz_min = radio_trim - _dead_zone;
    int16_t dz_max = radio_trim + _dead_zone;
    float ret;
    if (radio_in < dz_min && dz_min > radio_min) {
        ret = _reverse * (float)(radio_in - dz_min) / (float)(dz_min - radio_min);
    } else if (radio_in > dz_max && radio_max > dz_max) {
        ret = _reverse * (float)(radio_in - dz_max) / (float)(radio_max  - dz_max);
    } else {
        ret = 0;
    }
    return constrain_float(ret, -1.0f, 1.0f);
}

/*
  get percentage input from 0 to 100. This ignores the trim value.
 */
uint8_t
RC_Channel::percent_input()
{
    if (radio_in <= radio_min) {
        return _reverse==-1?100:0;
    }
    if (radio_in >= radio_max) {
        return _reverse==-1?0:100;
    }
    uint8_t ret = 100.0f * (radio_in - radio_min) / (float)(radio_max - radio_min);
    if (_reverse == -1) {
        ret = 100 - ret;
    }
    return ret;
}

float
RC_Channel::norm_output()
{
    int16_t mid = (radio_max + radio_min) / 2;
    float ret;
    if(radio_out < mid)
        ret = (float)(radio_out - mid) / (float)(mid - radio_min);
    else
        ret = (float)(radio_out - mid) / (float)(radio_max  - mid);
    if (_reverse == -1) {
	    ret = -ret;
    }
    return ret;
}

void RC_Channel::output() const
{
    rcout.write(_ch_out, radio_out);
}

void RC_Channel::output_trim() const
{
    rcout.write(_ch_out, radio_trim);
}

void RC_Channel::output_trim_all()
{
    for (uint8_t i=0; i<RC_MAX_CHANNELS; i++) {
        if (rc_ch[i] != NULL) {
            rc_ch[i]->output_trim();
        }
    }
}

/*
  setup the failsafe value to the trim value for all channels
 */
void RC_Channel::setup_failsafe_trim_all()
{
    for (uint8_t i=0; i<RC_MAX_CHANNELS; i++) {
        if (rc_ch[i] != NULL) {
//            rcout.set_failsafe_pwm(1U<<i, rc_ch[i]->radio_trim);
        }
    }
}

void
RC_Channel::input()
{
    radio_in = rcin.read(_ch_out);
}

uint16_t RC_Channel::read()
{
    return rcin.read(_ch_out);
}

void
RC_Channel::enable_out()
{
    rcout.enable_ch(_ch_out);
}

void
RC_Channel::disable_out()
{
    rcout.disable_ch(_ch_out);
}

RC_Channel *RC_Channel::rc_channel(uint8_t i)
{
    if (i >= RC_MAX_CHANNELS) {
        return NULL;
    }
    return rc_ch[i];
}

// return a limit PWM value
uint16_t RC_Channel::get_limit_pwm(LimitValue limit) const
{
    switch (limit) {
    case RC_CHANNEL_LIMIT_TRIM:
        return radio_trim;
    case RC_CHANNEL_LIMIT_MAX:
        return get_reverse() ? radio_min : radio_max;
    case RC_CHANNEL_LIMIT_MIN:
        return get_reverse() ? radio_max : radio_min;
    }
    // invalid limit value, return trim
    return radio_trim;
}
