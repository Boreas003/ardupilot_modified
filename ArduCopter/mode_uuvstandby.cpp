#include "Copter.h"

/*
 * Init and run calls for stand-by mode
 */

// stand-by_run - everything stops
// should be called at 100hz or more
void ModeStandby::run()
{
	motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
}
