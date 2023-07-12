#include "Copter.h"
#include <RC_Channel/RC_Channel.h>

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void ModeStabilize::run()
{
	// two different attitude controllers
	RC_Channel *ctry = rc().channel(int8_t(6)); // channel 7
	int16_t try_c;
	try_c = ctry->get_radio_in();

	float target_roll, target_pitch;
	_get_depth_reference = true;

	if (try_c > 1500) {
		// apply simple mode transform to pilot inputs
		update_simple_mode();

		// convert pilot input to lean angles
		get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);
	}
	else {
		// apply simple mode transform to pilot inputs
		update_simple_mode2();

		// convert pilot input to lean angles

		get_pilot_desired_lean_angles2(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);
	}

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    if (!motors->armed()) {
        // Motors should be Stopped
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    } else if (copter.ap.throttle_zero) {
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // call attitude controller

    // backward mode
	RC_Channel *cback = rc().channel(int8_t(12)); // channel 13 from 988 to 2012
	int16_t back;
	back = cback->get_radio_in();
	if (back > 1500){
		attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_uuv(-1.0f*target_roll, -1.0f*target_pitch, target_yaw_rate);
	}
	else {
		attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_uuv(target_roll, target_pitch, target_yaw_rate);
	}

    // output pilot's throttle
    attitude_control->set_throttle_out(get_pilot_desired_throttle(),
                                       false,
                                       g.throttle_filt);
}
