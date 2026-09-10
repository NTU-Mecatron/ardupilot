#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>

/*
    Speed controller for torpedo-shaped AUVs.
    Adjust throttle based on PI (w FF) control of speed error.
*/
class AP_SpeedController {
public:
    // Constructor
    AP_SpeedController();

    // Parameter definitions
    static const struct AP_Param::GroupInfo var_info[];

    /// Update speed controller with target speed and current speed to compute the desired throttle
    /// Must be called at minimum 50Hz
    void update(float target_speed, float current_speed);

    /// Get desired throttle pct, to be used by the motor controller
    /// @return  Desired throttle pct (-100 to 100)
    float get_throttle_demand() const { return _desired_throttle * 100.0f; }

    /// Reset integral gain
    void reset_I() { _pid_speed.reset_I(); }

    /// Get pid info
    const AP_PIDInfo& get_pid_info(void) const { return _pid_info; }

private:
    // PI controller (w FF): speed error -> throttle
    AC_PID _pid_speed{0.2f, 0.01f, 0.0f, 0.1f, 0.2f, 5.0f, 5.0f, 0.0f};

    // State variables
    float   _desired_throttle;      // Desired throttle in percent
    uint64_t _update_last_usec;         // Time of last update in microseconds

    AP_PIDInfo _pid_info;               // PID info for external access
};
