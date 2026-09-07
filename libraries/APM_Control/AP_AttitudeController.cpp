/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//	Originally by Jon Challinger, modified by Paul Riseborough (roll/pitch/yaw controllers)
//  Merged into a single axis-independent controller for torpedo-shaped AUVs

#include <AP_HAL/AP_HAL.h>
#include "AP_AttitudeController.h"
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Scheduler/AP_Scheduler.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_AttitudeController::var_info[] = {

    // @Param: _TCONST
    // @DisplayName: Attitude axis time constant
    // @Description: Time constant in seconds from demanded to achieved axis angle. Most models respond well to 0.5. May be reduced for faster responses, but setting lower than a model can achieve will not help.
    // @Range: 0.4 1.0
    // @Units: s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_TCONST", 1, AP_AttitudeController, gains.tau, 0.5f),

    // @Param: _MAX_RATE
    // @DisplayName: Attitude axis max rate
    // @Description: This sets the maximum positive rate that the attitude controller will demand (degrees/sec) in angle stabilized modes. Setting it to zero disables this limit.
    // @Range: 0 180
    // @Units: deg/s
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_MAX_RATE", 2, AP_AttitudeController, gains.rmax_pos, 30),

    // @Param: _RATE_P
    // @DisplayName: Attitude axis rate controller P gain
    // @Description: Rate controller P gain. Corrects in proportion to the difference between the desired rate vs actual rate. Note that rate = radians(rate) * scaler * scaler.
    // @Range: 0.08 0.35
    // @Increment: 0.005
    // @User: Standard

    // @Param: _RATE_I
    // @DisplayName: Attitude axis rate controller I gain
    // @Description: Rate controller I gain. Corrects long-term difference in desired rate vs actual rate
    // @Range: 0.01 0.6
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_IMAX
    // @DisplayName: Attitude axis rate controller I gain maximum
    // @Description: Rate controller I gain maximum. Constrains the maximum that the I term will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_D
    // @DisplayName: Attitude axis rate controller D gain
    // @Description: Rate controller D gain. Compensates for short-term change in desired rate vs actual rate
    // @Range: 0.001 0.03
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FF
    // @DisplayName: Attitude axis rate controller feed forward
    // @Description: Rate controller feed forward
    // @Range: 0 3.0
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FLTT
    // @DisplayName: Attitude axis rate controller target frequency in Hz
    // @Description: Rate controller target frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTE
    // @DisplayName: Attitude axis rate controller error frequency in Hz
    // @Description: Rate controller error frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTD
    // @DisplayName: Attitude axis rate controller derivative frequency in Hz
    // @Description: Rate controller derivative frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_SMAX
    // @DisplayName: Attitude axis slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: _RATE_PDMX
    // @DisplayName: Attitude axis rate controller PD sum maximum
    // @Description: Rate controller PD sum maximum. The maximum/minimum value that the sum of the P and D term can output
    // @Range: 0 1
    // @Increment: 0.01

    // @Param: _RATE_D_FF
    // @DisplayName: Attitude axis Derivative FeedForward Gain
    // @Description: FF D Gain which produces an output that is proportional to the rate of change of the target
    // @Range: 0 0.03
    // @Increment: 0.001
    // @User: Advanced

    // @Param: _RATE_NTF
    // @DisplayName: Attitude axis Target notch filter index
    // @Description: Target notch filter index
    // @Range: 1 8
    // @User: Advanced

    // @Param: _RATE_NEF
    // @DisplayName: Attitude axis Error notch filter index
    // @Description: Error notch filter index
    // @Range: 1 8
    // @User: Advanced

    AP_SUBGROUPINFO(rate_pid, "_RATE_", 9, AP_AttitudeController, AC_PID),

    AP_GROUPEND
};

AP_AttitudeController::AP_AttitudeController(const AP_FixedWing &parms, AP_AutoTune::ATType axis)
    : aparm(parms), _axis(axis)
{
    AP_Param::setup_object_defaults(this, var_info);
    rate_pid.set_slew_limit_scale(45);
}

float AP_AttitudeController::_measured_rate(void) const
{
    const Vector3f &gyro = AP::ahrs().get_gyro();
    switch (_axis) {
        case AP_AutoTune::AUTOTUNE_ROLL:
            return gyro.x;
        case AP_AutoTune::AUTOTUNE_PITCH:
            return gyro.y;
        case AP_AutoTune::AUTOTUNE_YAW:
            return gyro.z;
    }
    return 0;
}

float AP_AttitudeController::_limit_rate(float rate) const
{
    if (!gains.rmax_pos) {
        return rate;
    } else if (rate < -gains.rmax_pos) {
        return -gains.rmax_pos;
    } else if (rate > gains.rmax_pos) {
        return gains.rmax_pos;
    }
    return rate;
}

float AP_AttitudeController::get_rate_out(float desired_rate, float scaler, bool disable_integrator)
{
    const float dt = AP::scheduler().get_loop_period_s();
    bool limit_I = fabsf(_last_out) >= 45 || disable_integrator;
    const float rate = _measured_rate();
    desired_rate = _limit_rate(desired_rate);

    // the P and I elements are scaled by sq(scaler). To use an
    // unmodified AC_PID object we scale the inputs and calculate FF separately
    //
    // note that we run AC_PID in radians so that the normal scaling
    // range for IMAX in AC_PID applies (usually an IMAX value less than 1.0)
    rate_pid.update_all(radians(desired_rate) * scaler * scaler, rate * scaler * scaler, dt, limit_I);

    // FF should be scaled by scaler, but since we have scaled
    // the AC_PID target above by scaler*scaler we need to instead
    // divide by scaler to get the right scaling
    const float ff = degrees(ff_scale * rate_pid.get_ff() / scaler);
    ff_scale = 1.0;

    if (disable_integrator) {
        rate_pid.reset_I();
    }

    // convert AC_PID info object to same scale as old controller
    _pid_info = rate_pid.get_pid_info();
    auto &pinfo = _pid_info;

    const float deg_scale = degrees(1);
    pinfo.FF = ff;
    pinfo.P *= deg_scale;
    pinfo.I *= deg_scale;
    pinfo.D *= deg_scale;
    pinfo.DFF *= deg_scale;
    pinfo.limit = limit_I;

    // fix the logged target and actual values to not have the scalers applied
    pinfo.target = desired_rate;
    pinfo.actual = degrees(rate);

    // sum components
    float out = pinfo.FF + pinfo.P + pinfo.I + pinfo.D + pinfo.DFF;

    // remember the last output to trigger the I limit
    _last_out = out;

    if (autotune != nullptr && autotune->running) {
        // fake up an angular error based on a notional time constant of 0.5s
        angle_err_deg = desired_rate * gains.tau;
        autotune->update(pinfo, scaler, angle_err_deg);
    }

    // output is scaled to notional centidegrees of deflection
    return constrain_float(out * 100, -4500, 4500);
}

float AP_AttitudeController::get_servo_out(int32_t angle_err, float scaler, bool disable_integrator)
{
    if (gains.tau < 0.05f) {
        gains.tau.set(0.05f);
    }

    // Calculate the desired rate (deg/sec) from the angle error
    angle_err_deg = angle_err * 0.01f;
    float desired_rate = angle_err_deg / gains.tau;

    return get_rate_out(desired_rate, scaler, disable_integrator);
}

void AP_AttitudeController::autotune_start(void)
{
    if (autotune == nullptr) {
        autotune = new AP_AutoTune(gains, _axis, aparm, rate_pid);
        if (autotune == nullptr) {
            if (!failed_autotune_alloc) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AutoTune: failed allocation");
            }
            failed_autotune_alloc = true;
        }
    }
    if (autotune != nullptr) {
        autotune->start();
    }
}

void AP_AttitudeController::autotune_restore(void)
{
    if (autotune != nullptr) {
        autotune->stop();
    }
}
