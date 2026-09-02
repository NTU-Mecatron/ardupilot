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
    AP_AltitudeController(AP_AHRS &ahrs);

    // Parameter definitions
    static const struct AP_Param::GroupInfo var_info[];

    /// Target altitude in centimeters (positive = up)
    void set_target_altitude(int32_t target_alt_cm);

    /// Get latest target altitude in centimeters (positive = up)
    int32_t get_target_altitude() const { return _target_alt_cm; }

    /// Update altitude controller
    /// Must be called at minimum 50Hz
    /// Internally retrieves altitude from AHRS and computes desired pitch
    void update(float speed_scaler);

    /// Get desired pitch angle, to be used by the pitch controller
    /// @return  Desired pitch in centidegrees (positive = nose up)
    int32_t get_pitch_demand() const { return _desired_pitch_cd; }

    /// Reset integral gain
    void reset_I();

private:
    // AHRS reference for getting altitude and climb rate
    AP_AHRS &_ahrs;

    // Default PI controller with IMAX, estimated with 1m error == 15 deg pitch, I gain capped at 10 deg
    AC_PID _pid_alt{0.25f, 0.025f, 0.0f, 0.0f, 0.175f, 0.0f, 0.0f, 0.0f};

    // Parameters
    AP_Float _buoyancy_ff_deg;             // Buoyancy feedforward term (degrees)
    AP_Float _pitch_max;               // Maximum pitch angle (degrees)

    // State variables
    int32_t _target_alt_cm;             // Target altitude in cm
    int32_t _desired_pitch_cd;          // Desired pitch angle in centidegrees
    uint64_t _update_last_usec;         // Time of last update in microseconds
};
