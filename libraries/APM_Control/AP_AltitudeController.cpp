/// @file   AP_AltitudeController.cpp
/// @brief  Altitude controller for AUVs using AC_PID for depth control
/// @author ArduPilot Team

#include "AP_AltitudeController.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// Parameter information
const AP_Param::GroupInfo AP_AltitudeController::var_info[] = {
    // PID Params: altitude error -> pitch angle
    AP_SUBGROUPINFO(_pid_alt, "_ALT_", 0, AP_AltitudeController, AC_PID),

    // @Param: BUOY_FF
    // @DisplayName: Buoyancy feedforward pitch angle (degrees)
    // @Description: Pitch angle to counteract buoyancy (degrees) at typical operating speed (SCALING_SPEED); should be negative number to pitch down
    // @Increment: 1.0
    AP_GROUPINFO("BUOY_FF", 1, AP_AltitudeController, _buoyancy_ff_deg, -5.0f),

    // @Param: PITCH_MAX
    // @DisplayName: Maximum allowable pitch angle in magnitude (degrees)
    // @Description: Maximum allowable pitch angle in magnitude (degrees)
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
void AP_AltitudeController::set_altitude_target(int32 target_alt_cm)
{
    _target_alt_cm = target_alt_cm;
}

/// Reset the integral gain of the controller
void AP_AltitudeController::reset_I()
{
    _pid_alt.reset_I();
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

    // Update PID controller with altitude error
    // PID input is altitude in meters, output is desired pitch in radians
    bool limit_i_gain = true;
    float pitch_rad = _pid_alt.update_all(target_alt_m, current_alt_m, dt, limit_i_gain);

    // Actual feedforward pitch is dependent on speed; the higher speed, the lower pitch ff needed
    // speed_scaler = g.scaling_speed / current_speed, so we multiply by speed_scaler to adjust for current speed
    pitch_rad += radians(_buoyancy_ff_deg) * speed_scaler;

    // Constrain pitch angle to maximum
    float pitch_deg = degrees(pitch_rad);
    pitch_deg = constrain_float(pitch_deg, -_pitch_max, _pitch_max);

    // Convert to centidegrees
    _desired_pitch_cd = pitch_deg * 100.0f;
}