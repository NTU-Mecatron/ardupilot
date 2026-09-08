#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>
#include "AP_AutoTune.h"

/*
  Single rate/angle attitude axis controller, shared by roll, pitch and yaw.
  Convert desired angle/angle rate to servo commands for control surfaces.
  Designed for torp AUV, inspired by existing roll controller.
 */
class AP_AttitudeController
{
public:
    AP_AttitudeController(const AP_FixedWing &parms, AP_AutoTune::ATType axis);

    /* Do not allow copies */
    CLASS_NO_COPY(AP_AttitudeController);

    // get actuator output (centidegrees between -4500 and 4500) for direct rate control
    // desired_rate is in deg/sec. scaler is the surface/fin effectiveness scaler
    float get_rate_out(float desired_rate, float scaler, bool disable_integrator = false);

    // get actuator output from an angle error (centidegrees), via the rate controller
    float get_servo_out(int32_t angle_err, float scaler, bool disable_integrator);

    // setup a one loop FF scale multiplier. This replaces any previous scale applied
    // so should only be used when only one source of scaling is needed
    // preserved this for backward compatibility
    void set_ff_scale(float _ff_scale) { ff_scale = _ff_scale; }

    // reset I gain only
    void reset_I()
    {
        _pid_info.I = 0;
        rate_pid.reset_I();
    }

    // reset the entire rate PID controller
    void reset_rate_PID()
    {
        rate_pid.reset_I();
        rate_pid.reset_filter();
    }

    // rate control is always enabled - kept so existing call sites keep compiling
    bool enabled() const { return true; }
    bool rate_control_enabled() const { return true; }

    /*
      reduce the integrator, used when we have a low scale factor in a quadplane hover
    */
    void decay_I()
    {
        // this reduces integrator by 95% over 2s
        _pid_info.I *= 0.995f;
        rate_pid.set_integrator(rate_pid.get_i() * 0.995);
    }

    const AP_PIDInfo& get_pid_info(void) const { return _pid_info; }

    // set the PID notch sample rates
    void set_notch_sample_rate(float sample_rate) { rate_pid.set_notch_sample_rate(sample_rate); }

    // start/stop auto tuner
    void autotune_start(void);
    void autotune_restore(void);

    static const struct AP_Param::GroupInfo var_info[];

    // tuning accessors
    AP_Float &kP(void) { return rate_pid.kP(); }
    AP_Float &kI(void) { return rate_pid.kI(); }
    AP_Float &kD(void) { return rate_pid.kD(); }
    AP_Float &kFF(void) { return rate_pid.ff(); }
    AP_Float &tau(void) { return gains.tau; }
    AP_Int16 &max_rate(void) { return gains.rmax_pos; }

private:
    const AP_FixedWing &aparm;
    const AP_AutoTune::ATType _axis;

    AC_PID rate_pid{0.04, 0.15, 0, 0.15, 0.666, 3, 0, 12, 150, 1};

    float _last_out;
    float angle_err_deg;
    float ff_scale = 1.0;

    AP_AutoTune::ATGains gains;
    AP_AutoTune *autotune = nullptr;
    bool failed_autotune_alloc = false;

    AP_PIDInfo _pid_info;

    // return the measured body rate (rad/sec) for this axis
    float _measured_rate(void) const;

    // limit the desired rate to the maximum allowed by the controller
    float _limit_rate(float rate) const;
};
