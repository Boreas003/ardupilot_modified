/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  SRV_Channel_aux.cpp - handling of servo auxillary functions
 */
#include "SRV_Channel.h"

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>

#if NUM_SERVO_CHANNELS == 0
#pragma GCC diagnostic ignored "-Wtype-limits"
#endif

extern const AP_HAL::HAL& hal;

/// map a function to a servo channel and output it
void SRV_Channel::output_ch(void)
{
#ifndef HAL_BUILD_AP_PERIPH
    int8_t passthrough_from = -1;

    // take care of special function cases
    switch(function.get())
    {
    case k_manual:              // manual
        passthrough_from = ch_num;
        break;
    case k_rcin1 ... k_rcin16: // rc pass-thru
        passthrough_from = int8_t((int16_t)function - k_rcin1);
        break;
    case k_scripting1: // Thruster on the left
    	passthrough_from = 16;
    	break;
    case k_scripting2: // Thruster on the right
        	passthrough_from = 17;
        	break;
    case k_scripting3: // empty
        	passthrough_from = 18;
        	break;
    case k_scripting4: // empty
        	passthrough_from = 19;
        	break;
    }

    // control mode switch
    // proportional of throttle
    // difference pwm output control

    // ch6 thruster control mode
    // ch9 thruster direct control
    // ch14 difference control


    if (passthrough_from == 16){ // Thruster on the left

		RC_Channel *c1 = rc().channel(int8_t(5)); // mode switch 1495(up)-1750-2006
		RC_Channel *c2 = rc().channel(int8_t(8)); // 1366(right)-1622-mid1494
		RC_Channel *c3 = rc().channel(int8_t(13)); // 982-2006(right)
		RC_Channel *c4 = rc().channel(int8_t(2)); // 982-2006
		RC_Channel *c5 = rc().channel(int8_t(12)); // justify whether backward

		int16_t thrustermod; // mode
        int16_t thrusterctl; // control
        int16_t thrusterpdr; // pwm difference ratio
        int16_t throttle_value; // ch3 throttle
        int16_t back;
        float thrusteroutput;

		thrustermod = c1->get_radio_in();
		thrusterctl = c2->get_radio_in();
		thrusterpdr = c3->get_radio_in();
		throttle_value = c4->get_radio_in();
		back = c5->get_radio_in();

		if (c1 && c2 && c3 && c4 && c5) {
			if (SRV_Channels::passthrough_disabled()) {
				output_pwm = 1494;
			}
			else {
				if (thrustermod > 1800) { // switch down - default proportional 5% to 25%
					float throttle_pct = (1494.0f-thrusterctl)/128.0f*0.1f + 0.15f;
					float thruster_pdr = (-thrusterpdr+1494.0f)/512.0f*20.0f;
					if (back > 1500) { // backward
						thrusteroutput = (throttle_pct * (throttle_value-1549.0f)) - thruster_pdr + 1494.0f;
					}
					else { // forward
						thrusteroutput = -1.0f*(throttle_pct * (throttle_value-1549.0f)) + thruster_pdr + 1494.0f;
					}

					if (thrusteroutput < 1366) {
						output_pwm = 1366;
					}
					else if (thrusteroutput > 1622) {
						output_pwm = 1622;
					}
					else {
						output_pwm = (int)thrusteroutput;
					}
				}
				else if (thrustermod < 1700) {// switch up - individual control
					float thruster_pdr = (-thrusterpdr+1494.0f)/512.0f*20.0f;
					if (thrusterctl < 1494) { // forward
						thrusteroutput = thrusterctl +thruster_pdr;
					}
					else {
						thrusteroutput = thrusterctl - thruster_pdr;
					}
					if (thrusteroutput < 1366) {
						output_pwm = 1366;
					}
					else if (thrusteroutput > 1622) {
						output_pwm = 1622;
					}
					else {
						output_pwm = (int)thrusteroutput;
					}
				}
				else {
					output_pwm = 1494;
				}
			}
		}
		/*
    	RC_Channel *c1 = rc().channel(int8_t(0)); // channel roll
		RC_Channel *c2 = rc().channel(int8_t(1)); // channel pitch
		if (c1 && c2){
			if (SRV_Channels::passthrough_disabled()){
				output_pwm = (1495 - c1->get_radio_trim() - c2->get_radio_trim() + 1495 + 1495);
			} else {
				const int16_t radio_in = (1495 - c1->get_radio_trim() - c2->get_radio_trim() + 1495 + 1495);
				if (!ign_small_rcin_changes) {
					output_pwm = radio_in;
					previous_radio_in = radio_in;
				} else {
					if (abs(radio_in - previous_radio_in) > (1495 - c1->get_dead_zone() - c2->get_dead_zone() + 1495 + 1495)){
						output_pwm = radio_in;
						ign_small_rcin_changes = false;
					}
				}
			}
		}*/
	}


    if (passthrough_from == 17){ // Thruster on the right

    	int16_t thrustermod; // mode
		int16_t thrusterctl; // control
		int16_t thrusterpdr; // pwm difference ratio
		int16_t throttle_value; // ch3 throttle
		int16_t back;
		float thrusteroutput;

		RC_Channel *c1 = rc().channel(int8_t(5)); // mode switch 1495(up)-1750-2006
		RC_Channel *c2 = rc().channel(int8_t(8)); // 1366(right)-1622
		RC_Channel *c3 = rc().channel(int8_t(13)); // 982-2006(right)
		RC_Channel *c4 = rc().channel(int8_t(2)); // 982-2006
		RC_Channel *c5 = rc().channel(int8_t(12)); // justify whether backward

		thrustermod = c1->get_radio_in();
		thrusterctl = c2->get_radio_in();
		thrusterpdr = c3->get_radio_in();
		throttle_value = c4->get_radio_in();
		back = c5->get_radio_in();

		if (c1 && c2 && c3 && c4 && c5) {
			if (SRV_Channels::passthrough_disabled()) {
				output_pwm = 1494;
			}
			else {
				if (thrustermod > 1800) { // switch down - default proportional 5% to 25%
					float throttle_pct = (1494.0f-thrusterctl)/128.0f*0.1f + 0.15f;
					float thruster_pdr = (-thrusterpdr+1494.0f)/512.0f*20.0f;
					if (back > 1500) { // backward
						thrusteroutput = (throttle_pct * (throttle_value-1549.0f)) + thruster_pdr + 1494.0f;
					}
					else { // forward
						thrusteroutput = -1.0f*(throttle_pct * (throttle_value-1549.0f)) - thruster_pdr + 1494.0f;
					}

					if (thrusteroutput < 1366) {
						output_pwm = 1366;
					}
					else if (thrusteroutput > 1622) {
						output_pwm = 1622;
					}
					else {
						output_pwm = (int)thrusteroutput;
					}
				}
				else if (thrustermod < 1700) {// switch up - individual control
					float thruster_pdr = (-thrusterpdr+1494.0f)/512.0f*20.0f;
					if (thrusterctl < 1494) { // forward
						thrusteroutput = thrusterctl - thruster_pdr;
					}
					else { // backward
						thrusteroutput = thrusterctl + thruster_pdr;
					}
					if (thrusteroutput < 1366) {
						output_pwm = 1366;
					}
					else if (thrusteroutput > 1622) {
						output_pwm = 1622;
					}
					else {
						output_pwm = (int)thrusteroutput;
					}
				}
				else {
					output_pwm = 1494;
				}
			}
		}
		/*
    	RC_Channel *c1 = rc().channel(int8_t(0)); // channel roll
		RC_Channel *c2 = rc().channel(int8_t(1)); // channel pitch
		if (c1 && c2){
			if (SRV_Channels::passthrough_disabled()){
				output_pwm = (1495 - c1->get_radio_trim() - c2->get_radio_trim() + 1495 + 1495);
			} else {
				const int16_t radio_in = (1495 - c1->get_radio_trim() - c2->get_radio_trim() + 1495 + 1495);
				if (!ign_small_rcin_changes) {
					output_pwm = radio_in;
					previous_radio_in = radio_in;
				} else {
					if (abs(radio_in - previous_radio_in) > (1495 - c1->get_dead_zone() - c2->get_dead_zone() + 1495 + 1495)){
						output_pwm = radio_in;
						ign_small_rcin_changes = false;
					}
				}
			}
		}*/
	}


    // roll or pitch channel mid place 1495

    if (passthrough_from == 18){
		RC_Channel *c1 = rc().channel(int8_t(0)); // channel roll
		RC_Channel *c2 = rc().channel(int8_t(1)); // channel pitch
		if (c1 && c2){
			if (SRV_Channels::passthrough_disabled()){
				output_pwm = (1495 - c1->get_radio_trim() - c2->get_radio_trim() + 1495 + 1495);
			} else {
				const int16_t radio_in = (1495 - c1->get_radio_trim() - c2->get_radio_trim() + 1495 + 1495);
				if (!ign_small_rcin_changes) {
					output_pwm = radio_in;
					previous_radio_in = radio_in;
				} else {
					if (abs(radio_in - previous_radio_in) > (1495 - c1->get_dead_zone() - c2->get_dead_zone() + 1495 + 1495)){
						output_pwm = radio_in;
						ign_small_rcin_changes = false;
					}
				}
			}
		}
	}
    if (passthrough_from == 19){
		RC_Channel *c1 = rc().channel(int8_t(0)); // channel roll
		RC_Channel *c2 = rc().channel(int8_t(1)); // channel pitch
		if (c1 && c2){
			if (SRV_Channels::passthrough_disabled()){
				output_pwm = (c1->get_radio_trim() - c2->get_radio_trim() + 1495);
			} else {
				const int16_t radio_in = (c1->get_radio_in() - c2->get_radio_in() + 1495);
				if (!ign_small_rcin_changes) {
					output_pwm = radio_in;
					previous_radio_in = radio_in;
				} else {
					if (abs(radio_in - previous_radio_in) > (c1->get_dead_zone()-c2->get_dead_zone() + 1495)){
						output_pwm = radio_in;
						ign_small_rcin_changes = false;
					}
				}
			}
		}
	}



    if (passthrough_from != -1 && passthrough_from < 16) {
        // we are doing passthrough from input to output for this channel
        RC_Channel *c = rc().channel(passthrough_from);
        if (c) {
            if (SRV_Channels::passthrough_disabled()) {
                output_pwm = c->get_radio_trim();
            } else {
                const int16_t radio_in = c->get_radio_in();
                if (!ign_small_rcin_changes) {
                    output_pwm = radio_in;
                    previous_radio_in = radio_in;
                } else {
                    // check if rc input value has changed by more than the deadzone
                    if (abs(radio_in - previous_radio_in) > c->get_dead_zone()) {
                        output_pwm = radio_in;
                        ign_small_rcin_changes = false;
                    }
                }
            }
        }
    }
#endif // HAL_BUILD_AP_PERIPH

    if (!(SRV_Channels::disabled_mask & (1U<<ch_num))) {
        hal.rcout->write(ch_num, output_pwm);
    }
}

/*
  call output_ch() on all channels
 */
void SRV_Channels::output_ch_all(void)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        channels[i].output_ch();
    }
}

/*
  return the current function for a channel
*/
SRV_Channel::Aux_servo_function_t SRV_Channels::channel_function(uint8_t channel)
{
    if (channel < NUM_SERVO_CHANNELS) {
        return channels[channel].function;
    }
    return SRV_Channel::k_none;
}

/*
   setup a channels aux servo function
*/
void SRV_Channel::aux_servo_function_setup(void)
{
    if (type_setup) {
        return;
    }
    switch (function.get()) {
    case k_flap:
    case k_flap_auto:
    case k_egg_drop:
        set_range(100);
        break;
    case k_heli_rsc:
    case k_heli_tail_rsc:
    case k_motor_tilt:
    case k_boost_throttle:
    case k_thrust_out:
        set_range(1000);
        break;
    case k_aileron_with_input:
    case k_elevator_with_input:
    case k_aileron:
    case k_elevator:
    case k_dspoilerLeft1:
    case k_dspoilerLeft2:
    case k_dspoilerRight1:
    case k_dspoilerRight2:
    case k_rudder:
    case k_steering:
    case k_flaperon_left:
    case k_flaperon_right:
    case k_tiltMotorLeft:
    case k_tiltMotorRight:
    case k_tiltMotorRear:
    case k_tiltMotorRearLeft:
    case k_tiltMotorRearRight:
    case k_elevon_left:
    case k_elevon_right:
    case k_vtail_left:
    case k_vtail_right:
    case k_scripting1:
    case k_scripting2:
    case k_scripting3:
    case k_scripting4:
    case k_scripting5:
    case k_scripting6:
    case k_scripting7:
    case k_scripting8:
    case k_scripting9:
    case k_scripting10:
    case k_scripting11:
    case k_scripting12:
    case k_scripting13:
    case k_scripting14:
    case k_scripting15:
    case k_scripting16:
    case k_roll_out:
    case k_pitch_out:
    case k_yaw_out:
        set_angle(4500);
        break;
    case k_throttle:
    case k_throttleLeft:
    case k_throttleRight:
    case k_airbrake:
        // fixed wing throttle
        set_range(100);
        break;
    default:
        break;
    }
}

/// setup the output range types of all functions
void SRV_Channels::update_aux_servo_function(void)
{
    if (!channels) {
        return;
    }
    function_mask.clearall();

    for (uint16_t i = 0; i < SRV_Channel::k_nr_aux_servo_functions; i++) {
        functions[i].channel_mask = 0;
    }

    // set auxiliary ranges
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (!channels[i].valid_function()) {
            continue;
        }
        const uint16_t function = channels[i].function.get();
        channels[i].aux_servo_function_setup();
        function_mask.set(function);
        functions[function].channel_mask |= 1U<<i;
    }
    initialised = true;
}

/// Should be called after the the servo functions have been initialized
/// called at 1Hz
void SRV_Channels::enable_aux_servos()
{
    hal.rcout->set_default_rate(uint16_t(_singleton->default_rate.get()));

    update_aux_servo_function();

    // enable all channels that are set to a valid function. This
    // includes k_none servos, which allows those to get their initial
    // trim value on startup
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        // see if it is a valid function
        if (c.valid_function()) {
            hal.rcout->enable_ch(c.ch_num);
        }

        // output some servo functions before we fiddle with the
        // parameter values:
        if (c.function == SRV_Channel::k_min) {
            c.set_output_pwm(c.servo_min);
            c.output_ch();
        } else if (c.function == SRV_Channel::k_trim) {
            c.set_output_pwm(c.servo_trim);
            c.output_ch();
        } else if (c.function == SRV_Channel::k_max) {
            c.set_output_pwm(c.servo_max);
            c.output_ch();
        }
    }

    // propagate channel masks to the ESCS
    hal.rcout->update_channel_masks();

#if HAL_SUPPORT_RCOUT_SERIAL
    blheli_ptr->update();
#endif
}

/*
    for channels which have been marked as digital output then the
    MIN/MAX/TRIM values have no meaning for controlling output, as
    the HAL handles the scaling. We still need to cope with places
    in the code that may try to set a PWM value however, so to
    ensure consistency we force the MIN/MAX/TRIM to be consistent
    across all digital channels. We use a MIN/MAX of 1000/2000, and
    set TRIM to either 1000 or 1500 depending on whether the channel
    is reversible
*/
void SRV_Channels::set_digital_outputs(uint16_t dig_mask, uint16_t rev_mask) {
    digital_mask |= dig_mask;
    reversible_mask |= rev_mask;

    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        if (digital_mask & (1U<<i)) {
            c.servo_min.set(1000);
            c.servo_max.set(2000);
            if (reversible_mask & (1U<<i)) {
                c.servo_trim.set(1500);
            } else {
                c.servo_trim.set(1000);
            }
        }
    }
}

/// enable output channels using a channel mask
void SRV_Channels::enable_by_mask(uint16_t mask)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (mask & (1U<<i)) {
            hal.rcout->enable_ch(i);
        }
    }
}

/*
  set radio_out for all channels matching the given function type
 */
void SRV_Channels::set_output_pwm(SRV_Channel::Aux_servo_function_t function, uint16_t value)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].set_output_pwm(value);
            channels[i].output_ch();
        }
    }
}

/*
  set radio_out for all channels matching the given function type
  trim the output assuming a 1500 center on the given value
  reverses pwm output based on channel reversed property
 */
void
SRV_Channels::set_output_pwm_trimmed(SRV_Channel::Aux_servo_function_t function, int16_t value)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            int16_t value2;
            if (channels[i].get_reversed()) {
                value2 = 1500 - value + channels[i].get_trim();
            } else {
                value2 = value - 1500 + channels[i].get_trim();
            }
            channels[i].set_output_pwm(constrain_int16(value2,channels[i].get_output_min(),channels[i].get_output_max()));
            channels[i].output_ch();
          }
    }
}

/*
  set and save the trim value to current output for all channels matching
  the given function type
 */
void
SRV_Channels::set_trim_to_servo_out_for(SRV_Channel::Aux_servo_function_t function)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].servo_trim.set_and_save_ifchanged(channels[i].get_output_pwm());
        }
    }
}

/*
  copy radio_in to radio_out for a given function
 */
void
SRV_Channels::copy_radio_in_out(SRV_Channel::Aux_servo_function_t function, bool do_input_output)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            RC_Channel *c = rc().channel(channels[i].ch_num);
            if (c == nullptr) {
                continue;
            }
            channels[i].set_output_pwm(c->get_radio_in());
            if (do_input_output) {
                channels[i].output_ch();
            }
        }
    }
}

/*
  copy radio_in to radio_out for a channel mask
 */
void
SRV_Channels::copy_radio_in_out_mask(uint16_t mask)
{
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        if ((1U<<i) & mask) {
            RC_Channel *c = rc().channel(channels[i].ch_num);
            if (c == nullptr) {
                continue;
            }
            channels[i].set_output_pwm(c->get_radio_in());
        }
    }

}

/*
  setup failsafe value for an auxiliary function type to a Limit
 */
void
SRV_Channels::set_failsafe_pwm(SRV_Channel::Aux_servo_function_t function, uint16_t pwm)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        const SRV_Channel &c = channels[i];
        if (c.function == function) {
            hal.rcout->set_failsafe_pwm(1U<<c.ch_num, pwm);
        }
    }
}

/*
  setup failsafe value for an auxiliary function type to a Limit
 */
void
SRV_Channels::set_failsafe_limit(SRV_Channel::Aux_servo_function_t function, SRV_Channel::Limit limit)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        const SRV_Channel &c = channels[i];
        if (c.function == function) {
            uint16_t pwm = c.get_limit_pwm(limit);
            hal.rcout->set_failsafe_pwm(1U<<c.ch_num, pwm);
        }
    }
}

/*
  set radio output value for an auxiliary function type to a Limit
 */
void
SRV_Channels::set_output_limit(SRV_Channel::Aux_servo_function_t function, SRV_Channel::Limit limit)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        if (c.function == function) {
            uint16_t pwm = c.get_limit_pwm(limit);
            c.set_output_pwm(pwm);
            if (c.function == SRV_Channel::k_manual) {
                RC_Channel *cin = rc().channel(c.ch_num);
                if (cin != nullptr) {
                    // in order for output_ch() to work for k_manual we
                    // also have to override radio_in
                    cin->set_radio_in(pwm);
                }
            }
        }
    }
}

/*
  return true if a particular function is assigned to at least one RC channel
 */
bool
SRV_Channels::function_assigned(SRV_Channel::Aux_servo_function_t function)
{
    if (!initialised) {
        update_aux_servo_function();
    }
    return function_mask.get(uint16_t(function));
}

/*
  set servo_out and angle_min/max, then calc_pwm and output a
  value. This is used to move a AP_Mount servo
 */
void
SRV_Channels::move_servo(SRV_Channel::Aux_servo_function_t function,
                         int16_t value, int16_t angle_min, int16_t angle_max)
{
    if (!function_assigned(function)) {
        return;
    }
    if (angle_max <= angle_min) {
        return;
    }
    float v = float(value - angle_min) / float(angle_max - angle_min);
    v = constrain_float(v, 0.0f, 1.0f);
    for (uint8_t i = 0; i < NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        if (c.function == function) {
            float v2 = c.get_reversed()? (1-v) : v;
            uint16_t pwm = c.servo_min + v2 * (c.servo_max - c.servo_min);
            c.set_output_pwm(pwm);
        }
    }
}

/*
  set the default channel an auxiliary output function should be on
 */
bool SRV_Channels::set_aux_channel_default(SRV_Channel::Aux_servo_function_t function, uint8_t channel)
{
    if (function_assigned(function)) {
        // already assigned
        return true;
    }
    if (channels[channel].function != SRV_Channel::k_none) {
        if (channels[channel].function == function) {
            return true;
        }
        hal.console->printf("Channel %u already assigned function %u\n",
                            (unsigned)(channel + 1),
                            (unsigned)channels[channel].function.get());
        return false;
    }
    channels[channel].type_setup = false;
    channels[channel].function.set(function);
    channels[channel].aux_servo_function_setup();
    function_mask.set((uint16_t)function);
    if (SRV_Channel::valid_function(function)) {
        functions[function].channel_mask |= 1U<<channel;
    }
    return true;
}

// find first channel that a function is assigned to
bool SRV_Channels::find_channel(SRV_Channel::Aux_servo_function_t function, uint8_t &chan)
{
    if (!function_assigned(function)) {
        return false;
    }
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            chan = channels[i].ch_num;
            return true;
        }
    }
    return false;
}

// return channel that a function is assigned to
SRV_Channel *SRV_Channels::return_channel(SRV_Channel::Aux_servo_function_t function)
{
	for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++)
	{
		if (channels[i].function == function) {
			return &channels[i];
		}
	}
	return nullptr;
}

/*
  get a pointer to first auxillary channel for a channel function
*/
SRV_Channel *SRV_Channels::get_channel_for(SRV_Channel::Aux_servo_function_t function, int8_t default_chan)
{
    uint8_t chan;
    if (default_chan >= 0) {
        set_aux_channel_default(function, default_chan);
    }
    if (!find_channel(function, chan)) {
        return nullptr;
    }
    return &channels[chan];
}

void SRV_Channels::set_output_scaled(SRV_Channel::Aux_servo_function_t function, float value)
{
    if (SRV_Channel::valid_function(function)) {
        functions[function].output_scaled = value;
        SRV_Channel::have_pwm_mask &= ~functions[function].channel_mask;
    }
}

float SRV_Channels::get_output_scaled(SRV_Channel::Aux_servo_function_t function)
{
    if (SRV_Channel::valid_function(function)) {
        return functions[function].output_scaled;
    }
    return 0;
}

/*
  get mask of output channels for a function
 */
uint16_t SRV_Channels::get_output_channel_mask(SRV_Channel::Aux_servo_function_t function)
{
    if (!initialised) {
        update_aux_servo_function();
    }
    if (SRV_Channel::valid_function(function)) {
        return functions[function].channel_mask;
    }
    return 0;
}


// set the trim for a function channel to given pwm
void SRV_Channels::set_trim_to_pwm_for(SRV_Channel::Aux_servo_function_t function, int16_t pwm)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].servo_trim.set(pwm);
        }
    }
}

// set the trim for a function channel to min output of the channel honnoring reverse unless ignore_reversed is true
void SRV_Channels::set_trim_to_min_for(SRV_Channel::Aux_servo_function_t function, bool ignore_reversed)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].servo_trim.set((channels[i].get_reversed() && !ignore_reversed)?channels[i].servo_max:channels[i].servo_min);
        }
    }
}

/*
  set the default function for a channel
*/
void SRV_Channels::set_default_function(uint8_t chan, SRV_Channel::Aux_servo_function_t function)
{
    if (chan < NUM_SERVO_CHANNELS) {
        const SRV_Channel::Aux_servo_function_t old = channels[chan].function;
        channels[chan].function.set_default(function);
        if (old != channels[chan].function && channels[chan].function == function) {
            function_mask.set((uint16_t)function);
        }
    }
}

void SRV_Channels::set_esc_scaling_for(SRV_Channel::Aux_servo_function_t function)
{
    uint8_t chan;
    if (find_channel(function, chan)) {
        hal.rcout->set_esc_scaling(channels[chan].get_output_min(), channels[chan].get_output_max());
    }
}

/*
  auto-adjust channel trim from an integrator value. Positive v means
  adjust trim up. Negative means decrease
 */
void SRV_Channels::adjust_trim(SRV_Channel::Aux_servo_function_t function, float v)
{
    if (is_zero(v)) {
        return;
    }
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        if (function != c.function) {
            continue;
        }
        float change = c.reversed?-v:v;
        uint16_t new_trim = c.servo_trim;
        if (c.servo_max <= c.servo_min) {
            continue;
        }
        float trim_scaled = float(c.servo_trim - c.servo_min) / (c.servo_max - c.servo_min);
        if (change > 0 && trim_scaled < 0.6f) {
            new_trim++;
        } else if (change < 0 && trim_scaled > 0.4f) {
            new_trim--;
        } else {
            return;
        }
        c.servo_trim.set(new_trim);

        trimmed_mask |= 1U<<i;
    }
}

// get pwm output for the first channel of the given function type.
bool SRV_Channels::get_output_pwm(SRV_Channel::Aux_servo_function_t function, uint16_t &value)
{
    uint8_t chan;
    if (!find_channel(function, chan)) {
        return false;
    }
    if (!SRV_Channel::valid_function(function)) {
        return false;
    }
    channels[chan].calc_pwm(functions[function].output_scaled);
    value = channels[chan].get_output_pwm();
    return true;
}

// set output pwm to trim for the given function
void SRV_Channels::set_output_to_trim(SRV_Channel::Aux_servo_function_t function)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].set_output_pwm(channels[i].servo_trim);
        }
    }
}

/*
  get the normalised output for a channel function from the pwm value
  of the first matching channel
 */
float SRV_Channels::get_output_norm(SRV_Channel::Aux_servo_function_t function)
{
    uint8_t chan;
    if (!find_channel(function, chan)) {
        return 0;
    }
    if (SRV_Channel::valid_function(function)) {
        channels[chan].calc_pwm(functions[function].output_scaled);
    }
    return channels[chan].get_output_norm();
}

// set normalised output (-1 to 1 with 0 at mid point of servo_min/servo_max) for the given function
void SRV_Channels::set_output_norm(SRV_Channel::Aux_servo_function_t function, float value)
{
    if (!function_assigned(function)) {
        return;
    }
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        if (c.function == function) {
            c.set_output_norm(value);
        }
    }
}

/*
  limit slew rate for an output function to given rate in percent per
  second. This assumes output has not yet done to the hal
 */
void SRV_Channels::limit_slew_rate(SRV_Channel::Aux_servo_function_t function, float slew_rate, float dt)
{
    if (slew_rate <= 0) {
        // nothing to do
        return;
    }
    if (!SRV_Channel::valid_function(function)) {
        return;
    }
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        if (c.function == function) {
            c.calc_pwm(functions[function].output_scaled);
            uint16_t last_pwm = hal.rcout->read_last_sent(c.ch_num);
            if (last_pwm == c.get_output_pwm()) {
                continue;
            }
            uint16_t max_change = (c.get_output_max() - c.get_output_min()) * slew_rate * dt * 0.01f;
            if (max_change == 0 || dt > 1) {
                // always allow some change. If dt > 1 then assume we
                // are just starting out, and only allow a small
                // change for this loop
                max_change = 1;
            }
            c.set_output_pwm(constrain_int16(c.get_output_pwm(), last_pwm-max_change, last_pwm+max_change));
        }
    }
}

// call set_angle() on matching channels
void SRV_Channels::set_angle(SRV_Channel::Aux_servo_function_t function, uint16_t angle)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].set_angle(angle);
        }
    }    
}

// call set_range() on matching channels
void SRV_Channels::set_range(SRV_Channel::Aux_servo_function_t function, uint16_t range)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].set_range(range);
        }
    }
}

// set MIN parameter for a function
void SRV_Channels::set_output_min_max(SRV_Channel::Aux_servo_function_t function, uint16_t min_pwm, uint16_t max_pwm)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].function == function) {
            channels[i].set_output_min(min_pwm);
            channels[i].set_output_max(max_pwm);
        }
    }
}

// constrain to output min/max for function
void SRV_Channels::constrain_pwm(SRV_Channel::Aux_servo_function_t function)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        if (c.function == function) {
            c.set_output_pwm(constrain_int16(c.output_pwm, c.servo_min, c.servo_max));
        }
    }
}

/*
  upgrade SERVO* parameters. This does the following:

   - update to 16 bit FUNCTION from AP_Int8
*/
void SRV_Channels::upgrade_parameters(void)
{
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        // convert from AP_Int8 to AP_Int16
        c.function.convert_parameter_width(AP_PARAM_INT8);
    }
}

// set RC output frequency on a function output
void SRV_Channels::set_rc_frequency(SRV_Channel::Aux_servo_function_t function, uint16_t frequency_hz)
{
    uint16_t mask = 0;
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        SRV_Channel &c = channels[i];
        if (c.function == function) {
            mask |= (1U<<c.ch_num);
        }
    }
    if (mask != 0) {
        hal.rcout->set_freq(mask, frequency_hz);
    }
}
