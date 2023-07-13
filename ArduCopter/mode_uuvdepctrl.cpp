#include "Copter.h"
#include "mode.h"
bool _get_depth_reference;

/*
 * Init and run calls for stand-by mode
 */

// stand-by_run - everything stops
// should be called at 100hz or more
void ModeDepctrl::run()
{
	float target_roll, target_pitch;

	update_simple_mode2();

	// convert pilot input to lean angles
	get_pilot_desired_lean_angles_dep_ctrl(target_roll, target_pitch, _get_depth_reference, copter.aparm.angle_max, copter.aparm.angle_max);

	_get_depth_reference = false;

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

	attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_uuv(target_roll, target_pitch, target_yaw_rate);

	attitude_control->set_throttle_out(get_pilot_desired_throttle(),
	                                       false,
	                                       g.throttle_filt);
}
