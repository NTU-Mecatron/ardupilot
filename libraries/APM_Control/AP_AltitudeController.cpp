/// @file   AP_AltitudeController.cpp
/// @brief  Altitude controller for AUVs using AC_PID for depth control
/// @author ArduPilot Team

#include "AP_AltitudeController.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// Parameter information
const AP_Param::GroupInfo AP_AltitudeController::var_info[] = {
    // @Param: CTRL_P
    // @DisplayName: Altitude controller P gain
    // @Description: Altitude controller P gain. Converts the altitude error (meters) into a desired pitch angle (radians)
    // @Range: 0.1 0.3
    // @User: Standard

    // @Param: CTRL_I
    // @DisplayName: Altitude controller I gain
    // @Description: Altitude controller I gain. Integrates the altitude error (meters) over time to correct steady-state errors
    // @Range: 0.001 0.1
    // @User: Standard

    // @Param: CTRL_IMAX
    // @DisplayName: Altitude controller I maximum
    // @Description: Maximum value for the integral term (radians) to prevent windup
    // @Range: 0.1 0.2
    // @User: Standard
    AP_SUBGROUPINFO(_pid_alt, "CTRL_", 0, AP_AltitudeController, AC_PI),

    // @Param: BUOY_FF
    // @DisplayName: Buoyancy feedforward pitch angle (degrees)
    // @Description: Pitch angle to counteract buoyancy (degrees) at typical operating speed (SCALING_SPEED) in magnitude
    // @Increment: 1.0
    // @User: Standard
    AP_GROUPINFO("BUOYANCY_FF", 1, AP_AltitudeController, _buoyancy_ff_deg, 5.0f),

    AP_GROUPEND
};

// Constructor
AP_AltitudeController::AP_AltitudeController(AP_AHRS &ahrs) :
    _ahrs(ahrs),
    _target_alt_cm(0),
    _desired_pitch_cd(0),
    _update_last_usec(0)
{
    AP_Param::setup_object_defaults(this, var_info);
    _pid_info = _pid_alt.get_pid_info();
}

/// Update altitude controller
void AP_AltitudeController::update(float speed_scaler)
{
    // Calculate time since last update
    uint32_t now = AP_HAL::micros();
    float dt = (now - _update_last_usec) * 1.0e-6f;
    _update_last_usec = now;

    // Reset dt on first call or if dt is too large (indicates a long delay)
    if (dt > 1.0f) {
        dt = 0.02f;  // Assume 50Hz update rate
    }

    // Get current height above home from AHRS in meters (positive = up)
    // get_relative_position_D_home returns down distance from home, so negate it
    float current_alt_m = 0;
    _ahrs.get_relative_position_D_home(current_alt_m);
    current_alt_m *= -1.0f;  // Convert from down to up

    float target_alt_m = _target_alt_cm * 0.01f;

    // Update PI controller with altitude error (note that the arguments is measurement followed by target, different from AC_PID class)
    // PI input is altitude in meters, output is desired pitch in radians
    float pitch_rad = _pid_alt.update(current_alt_m * speed_scaler, target_alt_m * speed_scaler, dt);

    // Actual feedforward pitch is dependent on speed; the higher speed, the lower pitch ff needed
    // speed_scaler = g.scaling_speed / current_speed, so we multiply by speed_scaler to adjust for current speed
    pitch_rad += radians(_buoyancy_ff_deg) * speed_scaler;

    // Convert to centidegrees
    _desired_pitch_cd = degrees(pitch_rad) * 100.0f;

    // For logging purpose
    _pid_info = _pid_alt.get_pid_info();
}