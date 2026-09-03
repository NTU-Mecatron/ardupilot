#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

/*
  mode takeoff parameters
 */
const AP_Param::GroupInfo ModeTakeoff::var_info[] = {
    // @Param: ALT
    // @DisplayName: Takeoff mode altitude
    // @Description: This is the target altitude for TAKEOFF mode. Positive is up, so for AUV should be set to negative number.
    // @Range: -100 -0.1
    // @Increment: 0.1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("ALT", 1, ModeTakeoff, target_alt, -3.0),

    // @Param: MIN_SPEED
    // @DisplayName: Minimum takeoff speed
    // @Description: AUV need to reach this speed before pitching down to dive.
    // @Range: 0 10.0
    // @Increment: 0.1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("MIN_SPEED", 2, ModeTakeoff, min_takeoff_speed, 1.5),

    // @Param: SURFACE_PITCH
    // @DisplayName: Desired surface pitch
    // @Description: Target pitch for AUV to maintain when accelerating on the surface.
    // @Range: 0 15
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("SURFACE_PITCH", 3, ModeTakeoff, surface_pitch, 5),

    // @Param: IN_CIRCLE
    // @DisplayName: Takeoff in circle
    // @Description: Enable this to takeoff in circle if the altitude change is large.
    // @Range: 0 1
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("IN_CIRCLE", 4, ModeTakeoff, takeoff_in_circle, 0),

    AP_GROUPEND
};

ModeTakeoff::ModeTakeoff() :
    Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeTakeoff::_enter()
{
    takeoff_started = false;

    return true;
}

void ModeTakeoff::update()
{
    // don't setup waypoints if we dont have a valid position and home!
    if (!(plane.current_loc.initialised() && AP::ahrs().home_is_set())) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        return;
    }

    const float alt = target_alt;
    const float dist = target_dist;
    if (!takeoff_started) {
        const uint16_t altitude = plane.relative_ground_altitude(false,true);
        const float direction = degrees(ahrs.get_yaw());
        // see if we will skip takeoff as already flying
        if (plane.is_flying() && (millis() - plane.started_flying_ms > 10000U) && ahrs.groundspeed() > 3) {
            if (altitude >= alt) {
                gcs().send_text(MAV_SEVERITY_INFO, "Above TKOFF alt - loitering");
                plane.next_WP_loc = plane.current_loc;
                takeoff_started = true;
                plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Climbing to TKOFF alt then loitering");
                plane.next_WP_loc = plane.current_loc;
                plane.next_WP_loc.alt += ((alt - altitude) *100);
                plane.next_WP_loc.offset_bearing(direction, dist);
                takeoff_started = true;
                plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
            }
            // not flying so do a full takeoff sequence
        } else {
            // setup target waypoint and alt for loiter at dist and alt from start

            start_loc = plane.current_loc;
            plane.prev_WP_loc = plane.current_loc;
            plane.next_WP_loc = plane.current_loc;
            plane.next_WP_loc.alt += alt*100.0;
            plane.next_WP_loc.offset_bearing(direction, dist);

            plane.crash_state.is_crashed = false;

            plane.auto_state.takeoff_pitch_cd = level_pitch * 100;

            plane.set_flight_stage(AP_FixedWing::FlightStage::TAKEOFF);

            if (!plane.throttle_suppressed) {
                gcs().send_text(MAV_SEVERITY_INFO, "Takeoff to %.0fm for %.1fm heading %.1f deg",
                                alt, dist, direction);
                takeoff_started = true;
            }
        }
    }

    // we finish the initial level takeoff if we climb past
    // TKOFF_LVL_ALT or we pass the target location. The check for
    // target location prevents us flying forever if we can't climb
    // reset the loiter waypoint target to be correct bearing and dist
    // from starting location in case original yaw used to set it was off due to EKF
    // reset or compass interference from max throttle
    if (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF &&
        (plane.current_loc.alt - start_loc.alt >= level_alt*100 ||
         start_loc.get_distance(plane.current_loc) >= dist)) {
        // reset the target loiter waypoint using current yaw which should be close to correct starting heading
        const float direction = start_loc.get_bearing_to(plane.current_loc) * 0.01;
        plane.next_WP_loc = start_loc;
        plane.next_WP_loc.offset_bearing(direction, dist);
        plane.next_WP_loc.alt += alt*100.0;

        plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);

#if AP_FENCE_ENABLED
        plane.fence.auto_enable_fence_after_takeoff();
#endif
    }

    if (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100.0);
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
    } else {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
        //check if in long failsafe, if it is recall long failsafe now to get fs action via events call
        if (plane.long_failsafe_pending) {
        plane.long_failsafe_pending = false;
        plane.failsafe_long_on_event(FAILSAFE_LONG, ModeReason::MODE_TAKEOFF_FAILSAFE);
        }
    }
}

void ModeTakeoff::navigate()
{
    // Zero indicates to use WP_LOITER_RAD
    plane.update_loiter(0);
}

