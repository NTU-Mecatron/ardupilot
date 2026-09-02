/// @file   AP_AltitudeController.h
/// @brief  Altitude controller for AUVs using AC_PID for depth control
/// @author ArduPilot Team

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>

class AP_AltitudeController {
public:
    // Constructor
    AP_AltitudeController();

    // Parameter definitions
    static const struct AP_Param::GroupInfo var_info[];

    /// Set target altitude
    /// @param target_alt_cm    Target altitude in centimeters (positive = up)
    void set_altitude_target(float target_alt_cm);

    /// Get target altitude
    /// @return  Target altitude in centimeters
    float get_altitude_target() const { return _target_alt_cm; }

    /// Update altitude controller
    /// Must be called at minimum 50Hz
    /// Internally retrieves altitude from AHRS and computes desired pitch
    void update(float speed_scaler);

    /// Get desired pitch angle, to be used by the pitch controller
    /// @return  Desired pitch in centidegrees (positive = nose up)
    int32 get_pitch() const { return _desired_pitch_cd; }

    /// Load parameters from eeprom
    void load_gains();

    /// Reset the controller
    void reset();

private:
    // Calculate desired vertical acceleration and pitch based on altitude error
    void _calc_vertical_acc();

    // AHRS reference for getting altitude and climb rate
    AP_AHRS &_ahrs;

    // PID controller for vertical acceleration (depth control)
    AC_PID _pid_alt{0.05f, 0.01f, 0.005f, 0.0f, 1.0f, 0.02f};

    // Parameters
    AP_Float _buoyancy_ff;             // Buoyancy feedforward term (degrees)
    AP_Float _pitch_max;               // Maximum pitch angle (degrees)

    // State variables
    float   _target_alt_cm;             // Target altitude in cm
    int32   _desired_pitch_cd;          // Desired pitch angle in centidegrees
    uint64_t _update_last_usec;         // Time of last update in microseconds
};
