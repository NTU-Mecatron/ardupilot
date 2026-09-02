/// @file   AP_AltitudeController.cpp
/// @brief  Altitude controller for AUVs using AC_PID for depth control
/// @author ArduPilot Team

#include "AP_AltitudeController.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// Parameter information
const AP_Param::GroupInfo AP_AltitudeController::var_info[] = {
    // @Param: ALT_P
    // @DisplayName: Altitude controller gains
    // @Description: Gains for altitude control
    // @Range: 0.1 2.0
    // @Increment: 0.1
    AP_SUBGROUPINFO(_pid_alt, "ALT_", 0, AP_AltitudeController, AC_PID),

    // @Param: BUOY_FF
    // @DisplayName: Buoyancy feedforward pitch angle (magnitude in radians)
    // @Description: Pitch angle to counteract buoyancy (radians) at typical operating speed (SCALING_SPEED)
    // @Increment: 1.0
    AP_GROUPINFO("BUOY_FF", 1, AP_AltitudeController, _buoyancy_ff, 0.0f),

    // @Param: PITCH_MAX
    // @DisplayName: Maximum pitch angle
    // @Description: Maximum pitch angle for altitude control (degrees)
    // @Range: 5 20
    // @Increment: 1
    AP_GROUPINFO("PITCH_MAX", 2, AP_AltitudeController, _pitch_max, 15.0f),

    AP_GROUPEND
};

// Constructor
AP_AltitudeController::AP_AltitudeController() :
    _ahrs(AP_AHRS::get_singleton()),
    _target_alt_cm(0),
    _desired_pitch_cd(0),
    _update_last_usec(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/// Set target altitude
void AP_AltitudeController::set_altitude_target(float target_alt_cm)
{
    _target_alt_cm = target_alt_cm;
}

/// Load parameters
void AP_AltitudeController::load_gains()
{
    // Gains are loaded automatically via AP_PARAM system
    _pid_alt.load_gains();
}

/// Reset the controller
void AP_AltitudeController::reset()
{
    _pid_alt.reset_I();
    _desired_pitch_cd = 0;
}

/// Update altitude controller
void AP_AltitudeController::update(float speed_scaler)
{
    // Calculate time since last update
    uint64_t now = AP_HAL::micros64();
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

    // Update PID controller with altitude error
    // PID input is altitude in meters, output is desired pitch in radians
    float pitch_rad = _pid_alt.update_all(target_alt_m, current_alt_m, dt, true);

    // Actual feedforward pitch is dependent on speed; the higher speed, the lower pitch ff needed
    // speed_scaler = g.scaling_speed / current_speed, so we multiply by speed_scaler to adjust for current speed
    pitch_rad += _buoyancy_ff * speed_scaler;

    // Constrain pitch angle to maximum
    float pitch_deg = degrees(pitch_rad);
    pitch_deg = constrain_float(pitch_deg, -_pitch_max, _pitch_max);

    // Convert to centidegrees
    _desired_pitch_cd = pitch_deg * 100.0f;
}