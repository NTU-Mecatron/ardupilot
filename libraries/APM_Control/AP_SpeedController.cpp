
#include "AP_SpeedController.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// Parameter information
const AP_Param::GroupInfo AP_SpeedController::var_info[] = {
    // @Param: CTRL_P
    // @DisplayName: Speed controller P gain
    // @Description: Speed controller P gain. Converts the speed error (m/s) into a desired throttle (-1 to 1)
    // @Range: 0.1 0.5
    // @User: Standard

    // @Param: CTRL_I
    // @DisplayName: Speed controller I gain
    // @Description: Speed controller I gain. Integrates the speed error (m/s) over time to correct steady-state errors
    // @Range: 0.001 0.1
    // @User: Standard

    // @Param: CTRL_IMAX
    // @DisplayName: Speed controller I maximum
    // @Description: Maximum value for the integral term to prevent windup
    // @Range: 0.1 0.4
    // @User: Standard

    // @Param: CTRL_FF
    // @DisplayName: Speed controller feedforward
    // @Description: Feedforward term based on speed setpoint
    // @Range: 0.1 0.5
    // @User: Standard
    AP_SUBGROUPINFO(_pid_speed, "CTRL_", 0, AP_SpeedController, AC_PID),

    AP_GROUPEND
};

// Constructor
AP_SpeedController::AP_SpeedController(AP_AHRS &ahrs) :
    _ahrs(ahrs),
    _target_speed_ms(0.0f),
    _desired_throttle(0.0f),
    _update_last_usec(0)
{
    AP_Param::setup_object_defaults(this, var_info);
    _pid_info = _pid_speed.get_pid_info();
}

bool AP_SpeedController::get_forward_speed(float &speed) const
{
    // Implementation copied and simplified from AR_AttitudeControl
    Vector3f velocity;
    if (!_ahrs.get_velocity_NED(velocity))
        return false;

    // calculate forward speed velocity into body frame
    speed = velocity.x*_ahrs.cos_yaw() + velocity.y*_ahrs.sin_yaw();
    return true;
}

/// Update speed controller
void AP_SpeedController::update()
{
    // Get current speed estimate from AHRS
    float current_speed_ms;
    if (!get_forward_speed(current_speed_ms)) {
        _pid_speed.reset_filter();
        _desired_throttle = 0;
        return;
    }

    // Calculate time since last update
    uint32_t now = AP_HAL::micros();
    float dt = (now - _update_last_usec) * 1.0e-6f;
    _update_last_usec = now;

    // Reset dt on first call or if dt is too large (indicates a long delay)
    if (dt > 1.0f) {
        dt = 0.02f;  // Assume 50Hz update rate
    }

    // Update PID controller with speed error
    // PID input is speed in m/s, output is throttle (-1 to 1)
    bool limit_i_gain = true;
    _desired_throttle = _pid_speed.update_all(_target_speed_ms, current_speed_ms, dt, limit_i_gain);
    _desired_throttle += _pid_speed.get_ff();
    _desired_throttle = constrain_float(_desired_throttle, -1.0f, 1.0f);

    // For logging purpose
    _pid_info = _pid_speed.get_pid_info();
}
