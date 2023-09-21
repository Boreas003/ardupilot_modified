#include "Copter.h"

//
//  failsafe support
//  Andrew Tridgell, December 2011
//
//  our failsafe strategy is to detect main loop lockup and disarm the motors
//

static bool failsafe_enabled = false;
static uint16_t failsafe_last_ticks;
static uint32_t failsafe_last_timestamp;
static bool in_failsafe;

//
// failsafe_enable - enable failsafe
//
void Copter::failsafe_enable()
{
    failsafe_enabled = true;
    failsafe_last_timestamp = micros();
}

//
// failsafe_disable - used when we know we are going to delay the mainloop significantly
//
void Copter::failsafe_disable()
{
    failsafe_enabled = false;
}

//
//  failsafe_check - this function is called from the core timer interrupt at 1kHz.
//
void Copter::failsafe_check()
{
    uint32_t tnow = AP_HAL::micros();

    const uint16_t ticks = scheduler.ticks();
    if (ticks != failsafe_last_ticks) {
        // the main loop is running, all is OK
        failsafe_last_ticks = ticks;
        failsafe_last_timestamp = tnow;
        if (in_failsafe) {
            in_failsafe = false;
            AP::logger().Write_Error(LogErrorSubsystem::CPU, LogErrorCode::FAILSAFE_RESOLVED);
        }
        return;
    }

    if (!in_failsafe && failsafe_enabled && tnow - failsafe_last_timestamp > 2000000) {
        // motors are running but we have gone 2 second since the
        // main loop ran. That means we're in trouble and should
        // disarm the motors->
        in_failsafe = true;
        // reduce motors to minimum (we do not immediately disarm because we want to log the failure)
        if (motors->armed()) {
            motors->output_min();
        }

        AP::logger().Write_Error(LogErrorSubsystem::CPU, LogErrorCode::FAILSAFE_OCCURRED);
    }

    if (failsafe_enabled && in_failsafe && tnow - failsafe_last_timestamp > 1000000) {
        // disarm motors every second
        failsafe_last_timestamp = tnow;
        if(motors->armed()) {
            motors->armed(false);
            motors->output();
        }
    }
}

void Copter::failsafe_leak_check()
{
    bool status = leak_detector.get_status();

    // Do nothing if we are dry, or if leak failsafe action is disabled
    if (status == false || g.failsafe_leak == FS_LEAK_DISABLED) {
        if (failsafe.leak) {
            AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_LEAK, LogErrorCode::FAILSAFE_RESOLVED);
        }
        AP_Notify::flags.leak_detected = false;
        failsafe.leak = false;
        return;
    }

    AP_Notify::flags.leak_detected = status;

    uint32_t tnow = AP_HAL::millis();

    // We have a leak
    // Always send a warning every 20 seconds
    if (tnow > failsafe.last_leak_warn_ms + 20000) {
        failsafe.last_leak_warn_ms = tnow;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Leak Detected");
    }

    // Do nothing if we have already triggered the failsafe action, or if the motors are disarmed
    if (failsafe.leak) {
        return;
    }

    failsafe.leak = true;

    AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_LEAK, LogErrorCode::FAILSAFE_OCCURRED);

    //// Handle failsafe action
    //if (failsafe.leak && g.failsafe_leak == FS_LEAK_SURFACE && motors.armed()) {
        //set_mode(SURFACE, ModeReason::LEAK_FAILSAFE);
    //}
}
#if ADVANCED_FAILSAFE == ENABLED
/*
  check for AFS failsafe check
*/
void Copter::afs_fs_check(void)
{
    // perform AFS failsafe checks
#if AC_FENCE
    const bool fence_breached = fence.get_breaches() != 0;
#else
    const bool fence_breached = false;
#endif
    g2.afs.check(fence_breached, last_radio_update_ms);
}
#endif
