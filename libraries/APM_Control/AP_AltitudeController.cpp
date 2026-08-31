/// @file   AP_AltitudeController.cpp
/// @brief  Altitude controller for AUVs using AC_PID for depth control
/// @author ArduPilot Team

#include "AP_AltitudeController.h"
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

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
    _pid_alt(1.0f, 0.1f, 0.05f, 0.0f, 1.0f, 0.02f),
    _target_alt_cm(0),
    _desired_vertical_accel(0),
    _desired_pitch_cd(0),
    _alt_error_cm(0)
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
    _desired_pitch_cd = 0;
    _alt_error_cm = 0;
}

/// Update altitude controller
/// Called at 50Hz
void AP_AltitudeController::update(float current_alt_cm, float current_climb_rate_cms, float dt)
{
    // Calculate altitude error (positive = vehicle is below target)
    _alt_error_cm = _target_alt_cm - current_alt_cm;

    // Convert climb rate from cm/s to m/s
    float climb_rate_ms = current_climb_rate_cms * 0.01f;

    // PID controller computes desired vertical acceleration
    // Error is in cm, we need to convert to meters
    float alt_error_m = _alt_error_cm * 0.01f;
    
    _desired_vertical_accel = _pid_alt.get_pid(alt_error_m, dt);

    // Add buoyancy feedforward compensation
    // This helps maintain depth against buoyancy forces
    _desired_vertical_accel += _buoyancy_ff;

    // Constrain desired vertical acceleration
    _desired_vertical_accel = constrain_float(_desired_vertical_accel, 
                                             -_vertical_accel_max, 
                                             _vertical_accel_max);

    // Convert desired vertical acceleration to pitch angle
    // Using simple kinematic relationship: a = g * sin(pitch)
    // Therefore: pitch = asin(a / g)
    // For small angles and AUV speeds, we use: pitch_rad ≈ a / g
    const float GRAVITY = 9.81f;
    
    float pitch_rad = _desired_vertical_accel / GRAVITY;
    
    // Constrain pitch angle to maximum
    float pitch_max_rad = radians(_pitch_max);
    pitch_rad = constrain_float(pitch_rad, -pitch_max_rad, pitch_max_rad);
    
    // Convert to centidegrees
    _desired_pitch_cd = degrees(pitch_rad) * 100.0f;

    // Log data for debugging
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger && logger->logging_enabled()) {
        struct {
            LOG_PACKET_HEADER;
            uint32_t time_ms;
            float alt_target;
            float alt_current;
            float alt_error;
            float vert_accel;
            float pitch_desired;
            float climb_rate;
            float buoy_ff;
        } pkt = {
            LOG_PACKET_HEADER_INIT(LOG_ALTCTRL_MSG),
            time_ms : AP_HAL::millis(),
            alt_target : _target_alt_cm,
            alt_current : current_alt_cm,
            alt_error : _alt_error_cm,
            vert_accel : _desired_vertical_accel,
            pitch_desired : _desired_pitch_cd,
            climb_rate : climb_rate_ms,
            buoy_ff : _buoyancy_ff
        };
        logger->WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}
