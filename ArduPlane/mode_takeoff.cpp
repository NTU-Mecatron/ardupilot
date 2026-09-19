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

    // @Param: SPEED
    // @DisplayName: Minimum takeoff speed
    // @Description: AUV needs to reach this speed before pitching down to dive.
    // @Range: 0 10.0
    // @Increment: 0.1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("SPEED", 2, ModeTakeoff, takeoff_speed, 1.5),

    // @Param: LVL_PITCH
    // @DisplayName: Desired surface elevator deflection
    // @Description: When AUV speed is lower than AIRSPEED_MIN, the elevator will be hard-set to this deflection to avoid choppy pitch control.
    // @Range: 0 45
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("LVL_PITCH", 3, ModeTakeoff, surface_elevator, 20),

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
    // Do not enter takeoff if we are already running
    const float altitude = plane.adjusted_altitude_cm() * 0.01f;
    if (altitude < -1.0 && (millis() - plane.started_flying_ms > 5000U)) {
        return false;   // Luc_TODO: check what happen if return false
    }

    initial_heading_cd = -1;
    has_logged_reached_takeoff_speed = false;
    has_logged_reached_target_alt = false;

    // Set target speed
    plane.target_speed_ms = takeoff_speed;

    // Save prev and next waypoints
    plane.prev_WP_loc = plane.current_loc;
    plane.next_WP_loc = plane.current_loc;

    initial_heading_cd = wrap_360_cd(plane.ahrs.yaw_sensor);

    return true;
}

void ModeTakeoff::update()
{
    // Always calculate throttle to achieve given speed
    plane.calc_throttle();
    plane.calc_nav_pitch();
    plane.calc_nav_yaw_rate();    
}

void ModeTakeoff::navigate()
{
    // Check for optional timeout
    if (plane.g2.takeoff_timeout > 0) 
    {
        if (takeoff_start_ms == 0) {
            takeoff_start_ms = AP_HAL::millis();
        } else if (AP_HAL::millis() - takeoff_start_ms > (uint32_t)(1000U * plane.g2.takeoff_timeout)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Takeoff timeout at %.1f m/s", current_speed);
            plane.arming.disarm(AP_Arming::Method::TAKEOFFTIMEOUT, false);
            plane.set_mode(Mode::Number::MANUAL, ModeReason::FAILSAFE);
            return;
        }
    }

    if (!has_logged_reached_takeoff_speed)
    {
        if (plane.get_forward_speed() <= takeoff_speed * 0.9) {
            plane.next_WP_loc.alt = plane.prev_WP_loc.alt + 100.0f; // Set to 1.0m above water
        } else {
            plane.next_WP_loc.alt = plane.prev_WP_loc.alt + target_alt*100.0;
            has_logged_reached_takeoff_speed = true;
            const float tkoff_speed = takeoff_speed;
            gcs().send_text(MAV_SEVERITY_INFO, "Reached takeoff speed of %.1f", tkoff_speed);
        }
    }

    // Check if reached target alt
    if (!has_logged_reached_target_alt) {
        const float altitude = plane.adjusted_altitude_cm() * 0.01f;
        if (altitude >= target_alt) {
            plane.set_flight_stage(AP_FixedWing::FlightStage::TAKEOFF);
        } else {
            plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
            gcs().send_text(MAV_SEVERITY_INFO, "Current alt %.1f m, Reached target altitude of %.1f m", altitude, target_alt.get());
            has_logged_reached_target_alt = true;

            if (!takeoff_in_circle)
                plane.next_WP_loc = plane.current_loc;  // For loitering around current spot

#if AP_FENCE_ENABLED
            plane.fence.auto_enable_fence_after_takeoff();
#endif
        }
    }

    // Update nav controller based on flight stage
    if (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF) {
        if (takeoff_in_circle) {
            plane.update_loiter(0);     // Zero indicates to use WP_LOITER_RAD
        } else {    
            plane.nav_controller->update_heading_hold(initial_heading_cd);
        }      
    } else if (plane.flight_stage == AP_FixedWing::FlightStage::NORMAL) {
        plane.update_loiter(0);     // If we are still in takeoff mode, continue loitering
        initial_heading_cd = -1;    // Reset initial heading after takeoff is complete
    }
}