#include "Copter.h"

/*
 * Init and run calls for ground mode \\ ground 12/29
 */

// ground_run - runs the main stabilize controller ground_mode
// should be called at 100hz or more
void ModeGround::run()
{
	update_simple_mode();

	float target_yaw_rate = get_pilot_desired_yaw_rate(channel_roll->norm_input());

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
    	attitude_control->reset_yaw_target_and_rate();
		attitude_control->reset_rate_controller_I_terms();
		break;
    case AP_Motors::SpoolState::GROUND_IDLE:
    	attitude_control->reset_yaw_target_and_rate();
		attitude_control->reset_rate_controller_I_terms_smoothly();
		break;
    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
    	if (!motors->limit.throttle_lower) {
			set_land_complete(false);
		}
		break;
    case AP_Motors::SpoolState::SPOOLING_UP:
	case AP_Motors::SpoolState::SPOOLING_DOWN:
		// do nothing
		break;
	}

    attitude_control->input_euler_rate_yaw_uuv(target_yaw_rate);
    // the function above need to be modified to let roll and pitch get out from pid loop

    // output pilot's throttle
    attitude_control->set_throttle_out(get_pilot_desired_throttle(),
                                           false,
                                           g.throttle_filt);
}
