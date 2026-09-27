#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

/*
  mode takeoff parameters
 */
const AP_Param::GroupInfo ModeTakeoff::var_info[] = {
    // @Param: ALT
    // @DisplayName: Takeoff mode altitude
    // @Description: This is the target altitude for TAKEOFF mode. Positive is up, so for AUV should be set to negative number.
    // @Range: -100 -0.1
    // @Increment: 0.1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("ALT", 1, ModeTakeoff, target_alt, -3.0),

    // @Param: SPEED
    // @DisplayName: Minimum takeoff speed
    // @Description: AUV needs to reach this speed before pitching down to dive.
    // @Range: 0 10.0
    // @Increment: 0.1
    // @Units: m/s
    // @User: Standard
    AP_GROUPINFO("SPEED", 2, ModeTakeoff, takeoff_speed, 1.5),

    // @Param: PITCH
    // @DisplayName: Takeoff mode initial pitch
    // @Description: This is the target pitch for the initial takeoff to TKOFF_ALT. When we are sufficiently near to the target alt, pitch demand will be dictated by alt controller.
    // @Range: -20 0
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("PITCH", 3, ModeTakeoff, takeoff_pitch, -15),

    // @Param: GND_PITCH
    // @DisplayName: Takeoff run pitch demand
    // @Description: Degrees of pitch angle demanded during the takeoff run before speed reaches TKOFF_SPEED and after it has exceeded TKOFF_TDRAG_SPD1.
    // @Units: deg
    // @Range: 0.0 20.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("GND_PITCH", 4, ModeTakeoff, ground_pitch, 5),

    AP_GROUPEND
};

ModeTakeoff::ModeTakeoff() :
    Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeTakeoff::_enter()
{
    gcs().send_text(MAV_SEVERITY_WARNING, "Mode TakeOff is deprecated; you are prohibited from auto take-off without a mission.");
    return false;
}