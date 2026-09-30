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
    // @Range: 0.05 0.2
    // @User: Standard

    // @Param: CTRL_I
    // @DisplayName: Altitude controller I gain
    // @Description: Altitude controller I gain. Integrates the altitude error (meters) over time to correct steady-state errors
    // @Range: 0.001 0.1
    // @User: Standard

    // @Param: CTRL_D
    // @DisplayName: Altitude controller D gain
    // @Description: Altitude controller D gain. Reacts to the rate of change of the altitude error (meters/second) to provide damping
    // @Range: 0.01 0.2
    // @User: Standard

    // @Param: CTRL_IMAX
    // @DisplayName: Altitude controller I maximum
    // @Description: Maximum value for the integral term (radians) to prevent windup
    // @Range: 0.1 0.2
    // @User: Standard
    AP_SUBGROUPINFO(_pid_alt, "CTRL_", 0, AP_AltitudeController, AC_PID),

    // @Param: RES_ERROR
    // @DisplayName: Altitude residual error
    // @Description: Residual error in altitude control (meters). This is the magnitude of  altitude error at which I gain will be enabled. Outside of this, I term will not be unwind, but also will not accumulate. Should be positive value.
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("RES_ERROR", 1, AP_AltitudeController, _res_error, 1.0f),

    // @Param: BUOYANCY_FF
    // @DisplayName: Buoyancy feedforward pitch angle (degrees)
    // @Description: Pitch angle to counteract buoyancy (degrees) at typical operating speed (SCALING_SPEED). Should be negative number.
    // @Increment: 1.0
    // @User: Standard
    AP_GROUPINFO("BUOYANCY_FF", 2, AP_AltitudeController, _buoyancy_ff_deg, -2.0f),

    AP_GROUPEND
};

// Constructor
AP_AltitudeController::AP_AltitudeController() :
    _desired_pitch_cd(0),
    _update_last_usec(0)
{
    AP_Param::setup_object_defaults(this, var_info);
    _pid_info = _pid_alt.get_pid_info();
}

/// Update altitude controller
void AP_AltitudeController::update(float target_alt_cm, float current_alt_cm, float speed_scaler)
{
    // Calculate time since last update
    uint32_t now = AP_HAL::micros();
    float dt = (now - _update_last_usec) * 1.0e-6f;
    _update_last_usec = now;

    // Reset dt on first call or if dt is too large (indicates a long delay)
    if (dt > 1.0f) {
        dt = 0.02f;  // Assume 50Hz update rate
    }

    const float target_alt = target_alt_cm * 0.01f;
    const float current_alt = current_alt_cm * 0.01f;
    const float res_error = _res_error.get();
    const bool limit_I = res_error > 0.0f && fabsf(target_alt - current_alt) > res_error;

    // Update PID controller with altitude error
    // PID input is altitude in meters, output is desired pitch in radians
    // Pitch control is scaled by the speed factor
    // When speed is faster than scaling speed, we scale down the pitch quadratically with speed, else linearly
    const float scaler = (speed_scaler > 1.0) ? speed_scaler : speed_scaler * speed_scaler;
    float pitch_rad = _pid_alt.update_all(target_alt * scaler, current_alt * scaler, dt, limit_I);

    // Default feedforward pitch adjustment based on buoyancy (aka disturbance feedforward)
    // Notice that we do not use any built-in FF (aka setpoint feedforward) because it does not make sense
    if (fabsf(_buoyancy_ff_deg) > 0.1f) {
        pitch_rad += radians(_buoyancy_ff_deg) * scaler;
    }

    // Convert to centidegrees
    _desired_pitch_cd = degrees(pitch_rad) * 100.0f;

    // For logging purpose
    _pid_info = _pid_alt.get_pid_info();
    _pid_info.target = target_alt;
    _pid_info.actual = current_alt;
    _pid_info.error = _pid_info.target - _pid_info.actual;
}