/// @file   AP_AltitudeController.h
/// @brief  Altitude controller for AUVs using AC_PID for depth control
/// @author ArduPilot Team

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AC_PID/AC_PID.h>
#include <AP_Math/AP_Math.h>

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
    /// Must be called at 50Hz
    /// @param current_alt_cm   Current altitude in centimeters
    /// @param current_climb_rate_cms   Current climb rate in cm/s (positive = climbing)
    /// @param dt   Time delta in seconds
    void update(float current_alt_cm, float current_climb_rate_cms, float dt);

    /// Get desired vertical acceleration
    /// @return  Desired vertical acceleration in m/s^2 (positive = up)
    float get_desired_vertical_acceleration() const { return _desired_vertical_accel; }

    /// Get desired pitch angle
    /// @return  Desired pitch in centidegrees (positive = nose up)
    float get_desired_pitch() const { return _desired_pitch_cd; }

    /// Load parameters from eeprom
    void load_gains();

    /// Reset the controller
    void reset();

private:
    // PID controller for vertical acceleration (depth control)
    AC_PID  _pid_alt;

    // Parameters
    AP_Float    _buoyancy_ff;          // Buoyancy feedforward term (m/s^2)
    AP_Float    _pitch_max;            // Maximum pitch angle (degrees)
    AP_Float    _vertical_accel_max;   // Maximum vertical acceleration (m/s^2)

    // State variables
    float   _target_alt_cm;             // Target altitude in cm
    float   _desired_vertical_accel;    // Desired vertical acceleration in m/s^2
    float   _desired_pitch_cd;          // Desired pitch angle in centidegrees
    float   _alt_error_cm;              // Altitude error in cm
};
