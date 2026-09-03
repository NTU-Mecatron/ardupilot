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
    AP_SpeedController(AP_AHRS &ahrs);

    // Parameter definitions
    static const struct AP_Param::GroupInfo var_info[];

    /// Set target speed in m/s
    void set_target_speed(float target_speed_ms) { _target_speed_ms = target_speed_ms; }

    /// Get latest target speed in m/s
    float get_target_speed() const { return _target_speed_ms; }

    /// Update speed controller
    /// Must be called at minimum 50Hz
    /// Internally retrieves current speed from AHRS and computes desired throttle
    void update();

    /// Get desired throttle, to be used by the motor controller
    /// @return  Desired throttle (-100 to 100)
    float get_throttle_demand() const { return _desired_throttle * 100; }

    /// Reset integral gain
    void reset_I() { _pid_speed.reset_I(); }

    /// Get pid info
    const AP_PIDInfo& get_pid_info(void) const { return _pid_info; }

private:
    // Get speed along body forward axis
    bool get_forward_speed(float &speed) const;

    // AHRS reference for getting speed
    AP_AHRS &_ahrs;

    // PI controller (w FF): speed error -> throttle
    AC_PID _pid_speed{0.2f, 0.02f, 0.0f, 0.2f, 0.2f, 0.0f, 0.0f, 0.0f};

    // State variables
    float   _target_speed_ms;           // Target speed in m/s
    float   _desired_throttle;      // Desired throttle in percent
    uint64_t _update_last_usec;         // Time of last update in microseconds

    AP_PIDInfo _pid_info;               // PID info for external access
};
