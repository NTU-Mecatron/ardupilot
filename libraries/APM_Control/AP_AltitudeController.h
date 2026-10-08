#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>

/*
    Altitude controller for torpedo-shaped AUVs.
    Adjust pitch based on PI control of alt error, with FF term for buoyancy.
    Speed scale adjusted (faster == less pitch).
*/
class AP_AltitudeController {
public:
    // Constructor
    AP_AltitudeController();

    // Parameter definitions
    static const struct AP_Param::GroupInfo var_info[];

    /// Update altitude controller with alt error in meters (up is positive)
    /// Computed desired pitch is speed-scaled (faster speed requires less pitch)
    /// Must be called at minimum 50Hz
    void update(float target_alt_cm, float current_alt_cm, float speed_scaler);

    /// Get desired pitch angle, to be used by the pitch controller
    /// @return  Desired pitch in centidegrees (positive = nose up)
    int32_t get_pitch_demand() const { return _desired_pitch_cd; }

    /// Reset integral gain
    void reset_I() {_pid_alt.reset_I(); }

    /// Get pid info
    const AP_PIDInfo& get_pid_info(void) const { return _pid_info; }

private:
    AC_PID _pid_alt{0.1, 0.005, 0.05, 0.0, 0.07, 5.0, 0.0, 1.0};

    // Parameters
    AP_Float _buoyancy_ff_deg;             // Buoyancy feedforward term (degrees)
    AP_Float _res_error;               // Residual error in altitude control (meters) to enable I gain

    // State variables
    int32_t _desired_pitch_cd;          // Desired pitch angle in centidegrees
    uint64_t _update_last_usec;         // Time of last update in microseconds

    AP_PIDInfo _pid_info;                  // PID info for external access
};
