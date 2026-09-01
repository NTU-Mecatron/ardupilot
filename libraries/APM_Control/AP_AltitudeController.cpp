/// @file   AP_AltitudeController.cpp
/// @brief  Altitude controller for AUVs using AC_PID for depth control
/// @author ArduPilot Team

#include "AP_AltitudeController.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// Parameter information
const AP_Param::GroupInfo AP_AltitudeController::var_info[] = {
    // @Param: ALT_P
    // @DisplayName: Altitude controller P gain
    // @Description: Proportional gain for altitude control
    // @Range: 0.1 2.0
    // @Increment: 0.1
    AP_SUBGROUPINFO(_pid_alt, "ALT_", 0, AP_AltitudeController, AC_PID),

    // @Param: BUOY_FF
    // @DisplayName: Buoyancy feedforward
    // @Description: Buoyancy feedforward compensation (m/s^2)
    // @Range: -2.0 2.0
    // @Increment: 0.1
    AP_GROUPINFO("BUOY_FF", 1, AP_AltitudeController, _buoyancy_ff, 0.0f),

    // @Param: PITCH_MAX
    // @DisplayName: Maximum pitch angle
    // @Description: Maximum pitch angle for altitude control (degrees)
    // @Range: 5 45
    // @Increment: 1
    AP_GROUPINFO("PITCH_MAX", 2, AP_AltitudeController, _pitch_max, 25.0f),

    // @Param: ACCEL_MAX
    // @DisplayName: Maximum vertical acceleration
    // @Description: Maximum desired vertical acceleration (m/s^2)
    // @Range: 0.5 5.0
    // @Increment: 0.1
    AP_GROUPINFO("ACCEL_MAX", 3, AP_AltitudeController, _vertical_accel_max, 2.0f),

    AP_GROUPEND
};

// Constructor
AP_AltitudeController::AP_AltitudeController() :
    _ahrs(AP_AHRS::get_singleton()),
    _target_alt_cm(0),
    _desired_vertical_accel(0),
    _desired_pitch_rate_cd(0),
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
    _desired_vertical_accel = 0;
    _desired_pitch_rate_cd = 0;
}

/// Update altitude controller
/// Called at 50Hz
void AP_AltitudeController::update()
{
    _calc_vertical_acc();
    _calc_pitch_rate_from_vertical_acc();
}

void AP_AltitudeController::_calc_vertical_acc()
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
    // Input: target (desired altitude error = 0), measurement (current altitude error)
    // This computes: output = P*error + I*integral(error) + D*derivative(error)
    float pid_output = _pid_alt.update_all(target_alt_m, current_alt_m, dt, true);

    // Get individual PID components
    float ff_term = _pid_alt.get_ff();

    // Combine PID output with feedforward terms
    _desired_vertical_accel = pid_output + ff_term + _buoyancy_ff;

    // Constrain desired vertical acceleration
    _desired_vertical_accel = constrain_float(_desired_vertical_accel, 
                                             -_vertical_accel_max, 
                                             _vertical_accel_max);
}

/// Calculate desired pitch rate from vertical acceleration
/// Formula: pitch_rate = vertical_acc / speed
void AP_AltitudeController::_calc_pitch_rate_from_vertical_acc()
{
    const float GRAVITY = 9.81f;
    
    // Get current airspeed estimate
    float aspeed = 0;
    if (!_ahrs.airspeed_estimate(aspeed)) {
        // If no airspeed available, use a default minimum speed
        aspeed = 5.0f;  // m/s
    }
    
    // Ensure minimum airspeed to avoid division issues
    aspeed = MAX(aspeed, 1.0f);
    
    // Calculate pitch rate: pitch_rate (rad/s) = vertical_acc / speed
    float pitch_rate_rads = _desired_vertical_accel / aspeed;
    
    // Constrain pitch rate based on maximum pitch angle and speed
    // Maximum pitch rate should be limited to avoid aggressive maneuvers
    float max_pitch_rate_rads = radians(_pitch_max);  // Use pitch_max as rate limit proxy
    pitch_rate_rads = constrain_float(pitch_rate_rads, -max_pitch_rate_rads, max_pitch_rate_rads);
    
    // Convert to centidegrees/second
    _desired_pitch_rate_cd = degrees(pitch_rate_rads) * 100.0f;
}