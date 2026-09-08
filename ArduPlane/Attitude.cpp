#include "Plane.h"

/*
  return true if the current settings and mode should allow for stick mixing
 */
bool Plane::stick_mixing_enabled(void)
{
    if (!rc().has_valid_input()) {
        // never stick mix without valid RC
        return false;
    }
#if AP_FENCE_ENABLED
    const bool stickmixing = fence_stickmixing();
#else
    const bool stickmixing = true;
#endif
#if HAL_QUADPLANE_ENABLED
    if (control_mode == &mode_qrtl &&
        quadplane.poscontrol.get_state() >= QuadPlane::QPOS_POSITION1) {
        // user may be repositioning
        return false;
    }
    if (quadplane.in_vtol_land_poscontrol()) {
        // user may be repositioning
        return false;
    }
#endif
    if (control_mode->does_auto_throttle() && plane.control_mode->does_auto_navigation()) {
        // we're in an auto mode. Check the stick mixing flag
        if (g.stick_mixing != StickMixing::NONE &&
            g.stick_mixing != StickMixing::VTOL_YAW &&
            stickmixing) {
            return true;
        } else {
            return false;
        }
    }

    if (failsafe.rc_failsafe && g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // don't do stick mixing in FBWA glide mode
        return false;
    }

    // non-auto mode. Always do stick mixing
    return true;
}


/*
  this is the main roll stabilization function. It takes the
  previously set nav_roll calculates roll servo_out to try to
  stabilize the plane at the given roll
 */
void Plane::stabilize_roll()
{
#if HAL_QUADPLANE_ENABLED
    // Luc_TODO
#endif
    const float roll_out = rollController.get_servo_out(nav_roll_cd - ahrs.roll_sensor, get_speed_scaler(), false);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll_out);
}

/*
  this is the main pitch stabilization function. It takes the
  previously set nav_pitch and calculates servo_out values to try to
  stabilize the plane at the given attitude.
 */
void Plane::stabilize_pitch()
{
#if HAL_QUADPLANE_ENABLED
    // Luc_TODO
#endif
    const int32_t demanded_pitch = nav_pitch_cd + int32_t(g.pitch_trim * 100.0);
    const float pitch_out = pitchController.get_servo_out(demanded_pitch - ahrs.pitch_sensor, get_speed_scaler(), false);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch_out);
}

/*
  this gives the user control of the aircraft in stabilization modes
  using FBW style controls
 */
void Plane::stabilize_stick_mixing_fbw()
{
    if (!stick_mixing_enabled() ||
        control_mode == &mode_acro ||
        control_mode == &mode_fbwa ||
        control_mode == &mode_autotune ||
        control_mode == &mode_fbwb ||
        control_mode == &mode_cruise ||
#if HAL_QUADPLANE_ENABLED
        control_mode == &mode_qstabilize ||
        control_mode == &mode_qhover ||
        control_mode == &mode_qloiter ||
        control_mode == &mode_qland ||
        control_mode == &mode_qacro ||
#if QAUTOTUNE_ENABLED
        control_mode == &mode_qautotune ||
#endif
        !quadplane.allow_stick_mixing() ||
#endif  // HAL_QUADPLANE_ENABLED
        control_mode == &mode_training) {
        return;
    }
    // do FBW style stick mixing. We don't treat it linearly
    // however. For inputs up to half the maximum, we use linear
    // addition to the nav_roll and nav_pitch. Above that it goes
    // non-linear and ends up as 2x the maximum, to ensure that
    // the user can direct the plane in any direction with stick
    // mixing.
    float roll_input = channel_roll->norm_input_dz();
    if (roll_input > 0.5f) {
        roll_input = (3*roll_input - 1);
    } else if (roll_input < -0.5f) {
        roll_input = (3*roll_input + 1);
    }
    nav_roll_cd += roll_input * roll_limit_cd;
    nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);

    if ((control_mode == &mode_loiter) && (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL))) {
        // loiter is using altitude control based on the pitch stick, don't use it again here
        return;
    }

    float pitch_input = channel_pitch->norm_input_dz();
    if (pitch_input > 0.5f) {
        pitch_input = (3*pitch_input - 1);
    } else if (pitch_input < -0.5f) {
        pitch_input = (3*pitch_input + 1);
    }
    if (pitch_input > 0) {
        nav_pitch_cd += pitch_input * aparm.pitch_limit_max*100;
    } else {
        nav_pitch_cd += -(pitch_input * pitch_limit_min*100);
    }
    nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min*100, aparm.pitch_limit_max.get()*100);

    // rudder stick will control yaw rate, not yaw angle
    // hence, we use expo method to calculate the yaw rate input
    const int16_t max_yaw_rate = yawController.max_rate().get();
    const float rudd_expo = rudder_in_expo(true);
    const float yaw_rate_input = (rudd_expo/SERVO_MAX) * max_yaw_rate;
    nav_yaw_rate += yaw_rate_input;
    nav_yaw_rate = constrain_float(nav_yaw_rate, -max_yaw_rate, max_yaw_rate);
}


/*
  stabilize the yaw axis using rudder
 */
void Plane::stabilize_yaw()
{
    bool disable_integrator = false;
    if (control_mode == &mode_stabilize && rudder_input() != 0) {
        disable_integrator = true;
    }
    const float rudder_out = yawController.get_rate_out(nav_yaw_rate, get_speed_scaler(), disable_integrator);
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder_out);
}

/*
  main stabilization function for all 3 axes
 */
void Plane::stabilize()
{
    uint32_t now = AP_HAL::millis();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available()) {
        quadplane.transition->set_FW_roll_pitch(nav_pitch_cd, nav_roll_cd);
    }
#endif

    if (now - last_stabilize_ms > 2000) {
        // if we haven't run the rate controllers for 2 seconds then reset
        control_mode->reset_controllers();
    }
    last_stabilize_ms = now;

    if (control_mode == &mode_training ||
            control_mode == &mode_manual) {
        plane.control_mode->run();
#if AP_SCRIPTING_ENABLED
    } else if (nav_scripting_active()) {
        // scripting is in control of roll and pitch rates and throttle
        const float speed_scaler = get_speed_scaler();
        const float aileron = rollController.get_rate_out(nav_scripting.roll_rate_dps, speed_scaler);
        const float elevator = pitchController.get_rate_out(nav_scripting.pitch_rate_dps, speed_scaler);
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator);
        float rudder = 0;
        if (yawController.rate_control_enabled()) {
            rudder = nav_scripting.rudder_offset_pct * 45;
            if (nav_scripting.run_yaw_rate_controller) {
                rudder += yawController.get_rate_out(nav_scripting.yaw_rate_dps, speed_scaler, false);
            } else {
                yawController.reset_I();
            }
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, rudder);
#endif
    } else {
        plane.control_mode->run();
    }

    // Reset attitude controller integrators if vehicle is moving too slow, as there is very little control authority
    if (fabsf(get_forward_speed()) < aparm.airspeed_min) {
        rollController.reset_I();
        pitchController.reset_I();
        yawController.reset_I();
    }
}


void Plane::calc_throttle()
{
    const float commanded_throttle = speedController.get_throttle_demand();
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, commanded_throttle);
}

/*****************************************
* Calculate desired roll/pitch/yaw angles (in medium freq loop)
*****************************************/

/*
  get a new nav_pitch_cd from the alt-pitch controller
 */
void Plane::calc_nav_pitch()
{
    const int32_t commanded_pitch = alt_controller.get_pitch_demand();
    nav_pitch_cd = constrain_int32(commanded_pitch, aparm.pitch_limit_min.get()*100, aparm.pitch_limit_max.get()*100);
}


/*
  for torp auv, we always want zero roll
 */
void Plane::calc_nav_roll()
{
    nav_roll_cd = 0;
}

/*
  calculate desired yaw rate from desired lateral acceleration
 */
void Plane::calc_nav_yaw_rate()
{
    const float lat_acc = nav_controller->lateral_acceleration();
    nav_yaw_rate = degrees(lat_acc / MAX(get_forward_speed(), aparm.airspeed_min));
}

/*
  adjust nav_pitch_cd for STAB_PITCH_DOWN_CD. This is used to make
  keeping up good airspeed in FBWA mode easier, as the plane will
  automatically pitch down a little when at low throttle. It makes
  FBWA landings without stalling much easier.
 */
void Plane::adjust_nav_pitch_throttle(void)
{
    int8_t throttle = throttle_percentage();
    if (throttle >= 0 && throttle < aparm.throttle_cruise && flight_stage != AP_FixedWing::FlightStage::VTOL) {
        float p = (aparm.throttle_cruise - throttle) / (float)aparm.throttle_cruise;
        nav_pitch_cd -= g.stab_pitch_down * 100.0f * p;
    }
}


/*
  calculate a new aerodynamic_load_factor and limit nav_roll_cd to
  ensure that the load factor does not take us below the sustainable
  airspeed
 */
void Plane::update_load_factor(void)
{
    float demanded_roll = fabsf(nav_roll_cd*0.01f);
    if (demanded_roll > 85) {
        // limit to 85 degrees to prevent numerical errors
        demanded_roll = 85;
    }
    aerodynamic_load_factor = 1.0f / safe_sqrt(cosf(radians(demanded_roll)));

#if HAL_QUADPLANE_ENABLED
    if (quadplane.available() && quadplane.transition->set_FW_roll_limit(roll_limit_cd)) {
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
        return;
    }
#endif

    if (!aparm.stall_prevention) {
        // stall prevention is disabled
        return;
    }
    if (fly_inverted()) {
        // no roll limits when inverted
        return;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.tailsitter.active()) {
        // no limits while hovering
        return;
    }
#endif

    float max_load_factor = smoothed_airspeed / MAX(aparm.airspeed_min, 1);
    if (max_load_factor <= 1) {
        // our airspeed is below the minimum airspeed. Limit roll to
        // 25 degrees
        nav_roll_cd = constrain_int32(nav_roll_cd, -2500, 2500);
        roll_limit_cd = MIN(roll_limit_cd, 2500);
    } else if (max_load_factor < aerodynamic_load_factor) {
        // the demanded nav_roll would take us past the aerodynamic
        // load limit. Limit our roll to a bank angle that will keep
        // the load within what the airframe can handle. We always
        // allow at least 25 degrees of roll however, to ensure the
        // aircraft can be manoeuvred with a bad airspeed estimate. At
        // 25 degrees the load factor is 1.1 (10%)
        int32_t roll_limit = degrees(acosf(sq(1.0f / max_load_factor)))*100;
        if (roll_limit < 2500) {
            roll_limit = 2500;
        }
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit, roll_limit);
        roll_limit_cd = MIN(roll_limit_cd, roll_limit);
    }    
}
