#include "Plane.h"

/*
  reset the total loiter angle
 */
void Plane::loiter_angle_reset(void)
{
    loiter.sum_cd = 0;
    loiter.total_cd = 0;
    loiter.reached_target_alt = false;
    loiter.unable_to_acheive_target_alt = false;
}

/*
  update the total angle we have covered in a loiter. Used to support
  commands to do N circles of loiter
 */
void Plane::loiter_angle_update(void)
{
    static const int32_t lap_check_interval_cd = 3*36000;

    const int32_t target_bearing_cd = nav_controller->target_bearing_cd();
    int32_t loiter_delta_cd;
    const bool reached_target = reached_loiter_target();

    if (loiter.sum_cd == 0 && !reached_target) {
        // we don't start summing until we are doing the real loiter
        loiter_delta_cd = 0;
    } else if (loiter.sum_cd == 0) {
        // use 1 cd for initial delta
        loiter_delta_cd = 1;
        loiter.start_lap_alt_cm = current_loc.alt;
        loiter.next_sum_lap_cd = lap_check_interval_cd;
    } else {
        loiter_delta_cd = target_bearing_cd - loiter.old_target_bearing_cd;
    }

    loiter.old_target_bearing_cd = target_bearing_cd;
    loiter_delta_cd = wrap_180_cd(loiter_delta_cd);
    loiter.sum_cd += loiter_delta_cd * loiter.direction;

    bool reached_target_alt = false;

    if (reached_target) {
        // once we reach the position target we start checking the
        // altitude target
        bool terrain_status_ok = false;
#if AP_TERRAIN_AVAILABLE
        /*
          if doing terrain following then we check against terrain
          target, fetch the terrain information
        */
        float altitude_agl = 0;
        if (target_altitude.terrain_following) {
            if (terrain.status() == AP_Terrain::TerrainStatusOK &&
                terrain.height_above_terrain(altitude_agl, true)) {
                terrain_status_ok = true;
            }
        }
        if (terrain_status_ok &&
            fabsF(altitude_agl - target_altitude.terrain_alt_cm*0.01) < 5) {
            reached_target_alt = true;
        } else
#endif
        if (!terrain_status_ok && labs(current_loc.alt - target_altitude.amsl_cm) < 500) {
            reached_target_alt = true;
        }
    }

    if (reached_target_alt) {
        loiter.reached_target_alt = true;
        loiter.unable_to_acheive_target_alt = false;
        loiter.next_sum_lap_cd = loiter.sum_cd + lap_check_interval_cd;

    } else if (!loiter.reached_target_alt && labs(loiter.sum_cd) >= loiter.next_sum_lap_cd) {
        // check every few laps for scenario where up/downward inhibit you from loitering up/down for too long
        loiter.unable_to_acheive_target_alt = labs(current_loc.alt - loiter.start_lap_alt_cm) < 500;
        loiter.start_lap_alt_cm = current_loc.alt;
        loiter.next_sum_lap_cd += lap_check_interval_cd;
    }
}

//****************************************************************
// Function that will calculate the desired direction to fly and distance
//****************************************************************
void Plane::navigate()
{
    // do not navigate with corrupt data
    // ---------------------------------
    if (!have_position) {
        return;
    }

    if (next_WP_loc.lat == 0 && next_WP_loc.lng == 0) {
        return;
    }

    check_home_alt_change();

    // waypoint distance from plane
    // ----------------------------
    // auto_state.wp_distance = current_loc.get_distance(next_WP_loc);
    // auto_state.wp_proportion = current_loc.line_path_proportion(prev_WP_loc, next_WP_loc);
    // TECS_controller.set_path_proportion(auto_state.wp_proportion);
    // Luc_TODO

    // update total loiter angle
    loiter_angle_update();

    // control mode specific updates to navigation demands
    // ---------------------------------------------------
    control_mode->navigate();
}

void Plane::calc_gndspeed_undershoot()
{
    // Use the component of ground speed in the forward direction
    // This prevents flyaway if wind takes plane backwards
    Vector3f velNED;
    if (ahrs.have_inertial_nav() && ahrs.get_velocity_NED(velNED)) {
        const Matrix3f &rotMat = ahrs.get_rotation_body_to_ned();
        Vector2f yawVect = Vector2f(rotMat.a.x,rotMat.b.x);
        if (!yawVect.is_zero()) {
            yawVect.normalize();
            float gndSpdFwd = yawVect * velNED.xy();
            groundspeed_undershoot_is_valid = aparm.min_groundspeed > 0;
            groundspeed_undershoot = groundspeed_undershoot_is_valid ? (aparm.min_groundspeed*100 - gndSpdFwd*100) : 0;
        }
    } else {
        groundspeed_undershoot_is_valid = false;
        groundspeed_undershoot = 0;
    }
}

// method intended to be used by update_loiter
void Plane::update_loiter_update_nav(uint16_t radius)
{
#if HAL_QUADPLANE_ENABLED
    if (loiter.start_time_ms != 0 &&
        quadplane.guided_mode_enabled()) {
        if (!auto_state.vtol_loiter) {
            auto_state.vtol_loiter = true;
            // reset loiter start time, so we don't consider the point
            // reached till we get much closer
            loiter.start_time_ms = 0;
            quadplane.guided_start();
        }
        return;
    }
#endif

#if HAL_QUADPLANE_ENABLED
    const bool quadplane_qrtl_switch = (control_mode == &mode_rtl && quadplane.available() && quadplane.rtl_mode == QuadPlane::RTL_MODE::SWITCH_QRTL);
#else
    const bool quadplane_qrtl_switch = false;
#endif

    if ((loiter.start_time_ms == 0 &&
         (control_mode == &mode_auto || control_mode == &mode_guided) &&
         auto_state.crosstrack &&
         current_loc.get_distance(next_WP_loc) > radius*3) ||
        quadplane_qrtl_switch) {
        /*
          if never reached loiter point and using crosstrack and somewhat far away from loiter point
          navigate to it like in auto-mode for normal crosstrack behavior

          we also use direct waypoint navigation if we are a quadplane
          that is going to be switching to QRTL when it gets within
          RTL_RADIUS
        */
        nav_controller->update_waypoint(prev_WP_loc, next_WP_loc);
        return;
    }
    nav_controller->update_loiter(next_WP_loc, radius, loiter.direction);
}

void Plane::update_loiter(uint16_t radius)
{
    if (radius <= 1) {
        // if radius is <=1 then use the general loiter radius. if it's small, use default
        radius = (abs(aparm.loiter_radius) <= 1) ? LOITER_RADIUS_DEFAULT : abs(aparm.loiter_radius);
        if (next_WP_loc.loiter_ccw == 1) {
            loiter.direction = -1;
        } else {
            loiter.direction = (aparm.loiter_radius < 0) ? -1 : 1;
        }
    }

    // the radius actually being used by the controller is required by other functions
    loiter.radius = (float)radius;

    update_loiter_update_nav(radius);

    if (loiter.start_time_ms == 0) {
        if (reached_loiter_target() ||
            auto_state.wp_proportion > 1) {
            // we've reached the target, start the timer
            loiter.start_time_ms = millis();
            if (control_mode->is_guided_mode()) {
                // starting a loiter in GUIDED means we just reached the target point
                gcs().send_mission_item_reached_message(0);
            }
#if HAL_QUADPLANE_ENABLED
            if (quadplane.guided_mode_enabled()) {
                quadplane.guided_start();
            }
#endif
        }
    }
}

/*
  handle speed and height control in FBWB, CRUISE, and optionally, LOITER mode.
  In this mode the elevator is used to change target altitude. The
  throttle is used to change target airspeed or throttle
 */
void Plane::update_fbwb_speed_height(void)
{
    uint32_t now = micros();
    if (now - target_altitude.last_elev_check_us >= 100000) {
        // we don't run this on every loop as it would give too small granularity on quadplanes at 300Hz, and
        // give below 1cm altitude change, which would result in no climb or descent
        float dt = (now - target_altitude.last_elev_check_us) * 1.0e-6;
        dt = constrain_float(dt, 0.1, 0.15);

        target_altitude.last_elev_check_us = now;

        float elevator_input = channel_pitch->get_control_in() * (1/4500.0);

        if (g.flybywire_elev_reverse) {
            elevator_input = -elevator_input;
        }

        bool input_stop_climb = !is_positive(elevator_input) && is_positive(target_altitude.last_elevator_input);
        bool input_stop_descent = !is_negative(elevator_input) && is_negative(target_altitude.last_elevator_input);
        if (input_stop_climb || input_stop_descent) {
            // user elevator input reached or passed zero, lock in the current altitude
            set_target_altitude_current();
        }

        int32_t alt_change_cm = g.flybywire_climb_rate * elevator_input * dt * 100;
        change_target_altitude(alt_change_cm);

#if HAL_SOARING_ENABLED
        if (g2.soaring_controller.is_active()) {
            if (g2.soaring_controller.get_throttle_suppressed()) {
                // we're in soaring mode with throttle suppressed
                set_target_altitude_current();
            } else {
                // we're in soaring mode climbing back to altitude. Set target to SOAR_ALT_CUTOFF plus 10m to ensure we positively climb
                // through SOAR_ALT_CUTOFF, thus triggering throttle suppression and return to glide.
                target_altitude.amsl_cm = 100*plane.g2.soaring_controller.get_alt_cutoff() + 1000 + AP::ahrs().get_home().alt;
            }
        }
#endif

        target_altitude.last_elevator_input = elevator_input;
    }

    check_fbwb_altitude();

    altitude_error_cm = calc_altitude_error_cm();

    calc_throttle();
    calc_nav_pitch();
}

/*
  calculate the turn angle for the next leg of the mission
 */
void Plane::setup_turn_angle(void)
{
    int32_t next_ground_course_cd = mission.get_next_ground_course_cd(-1);
    if (next_ground_course_cd == -1) {
        // the mission library can't determine a turn angle, assume 90 degrees
        auto_state.next_turn_angle = 90.0f;
    } else {
        // get the heading of the current leg
        int32_t ground_course_cd = prev_WP_loc.get_bearing_to(next_WP_loc);

        // work out the angle we need to turn through
        auto_state.next_turn_angle = wrap_180_cd(next_ground_course_cd - ground_course_cd) * 0.01f;
    }
}

/*
  see if we have reached our loiter target
 */
bool Plane::reached_loiter_target(void)
{
#if HAL_QUADPLANE_ENABLED
    if (quadplane.in_vtol_auto()) {
        return auto_state.wp_distance < 3;
    }
#endif
    return nav_controller->reached_loiter_target();
}
