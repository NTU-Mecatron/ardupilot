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

    // @Param: SURFACE_ELEVATOR
    // @DisplayName: Desired surface elevator deflection
    // @Description: When AUV speed is lower than AIRSPEED_MIN, the elevator will be hard-set to this deflection to avoid choppy pitch control.
    // @Range: 0 45
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("SURFACE_ELEVATOR", 3, ModeTakeoff, surface_elevator, 20),

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
    const uint16_t altitude = plane.relative_ground_altitude(false,true);
    if (altitude < -1.0 && (millis() - plane.started_flying_ms > 5000U)) {
        return false;   // Luc_TODO: check what happen if return false
    }

    initial_heading_cd = -1;
    return true;
}

void ModeTakeoff::update()
{
    // don't setup waypoints if we dont have a valid position and home!
    if (!(plane.current_loc.initialised() && AP::ahrs().home_is_set())) {
        plane.arming.disarm(AP_Arming::Method::EKFFAILSAFE, false);
        return;
    }

    plane.calc_throttle();

    // If we are too slow, pitch and yaw will be heavily affected by waves so it is better to hardcode elevator and rudder
    if (current_speed < takeoff_speed) 
    {
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, surface_elevator * 100);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0.0);
    }
    else    // Run inner PID loops as per usual
    {
        plane.calc_nav_pitch();
        plane.calc_nav_yaw_ground();    // Luc_TODO: implement a general calc_nav_yaw
    }

    // Check if reached target alt (which should be a negative number)
    const uint16_t altitude = plane.relative_ground_altitude(false,true);
    if (altitude >= target_alt) {
        plane.set_flight_stage(AP_FixedWing::FlightStage::TAKEOFF);
    } else {
        plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
    }
}

void ModeTakeoff::run()
{
    // When not enough speed, let the update() method handle hardcode control of elevator and rudder
    if (current_speed >= takeoff_speed)
    {
        // Normal flight, run base class
        Mode::run();
    }
}

void ModeTakeoff::navigate()
{
    // We put this get_forward_speed in navigate so that it runs at a lower rate
    // As it is a relatively heavy function (compute cos and sin and stuff)
    if (!plane.speedController.get_forward_speed(current_speed))    
    {
        plane.arming.disarm(AP_Arming::Method::EKFFAILSAFE, false);
        return;
    }

    plane.speedController.set_target_speed(takeoff_speed * 1.1f);   // Set a slightly higher target speed for margin
    plane.alt_pitch_controller.set_target_altitude(target_alt * 100);   // Always set target depth, but update() method only pitch down if has reached sufficient speed

    if (takeoff_in_circle) {
        // Luc_TODO: update loiter with changing depth, or separate the axes
        plane.update_loiter(0);
    } else {    // Maintain heading when dive
        if (initial_heading_cd == -1) {
            initial_heading_cd = wrap_360_cd(plane.ahrs.yaw_sensor);
        }
        plane.nav_controller->update_heading_hold(initial_heading_cd);
    }

    if (plane.flight_stage == AP_FixedWing::FlightStage::NORMAL) {
        plane.update_loiter(0); // Luc_TODO: what do we do after done taking off?
        initial_heading_cd = -1;    // Reset initial heading after takeoff is complete
    }
}