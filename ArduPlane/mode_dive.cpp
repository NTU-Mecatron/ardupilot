#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

/*
  mode dive parameters
 */
const AP_Param::GroupInfo ModeDive::var_info[] = {
    // @Param: DEPTH
    // @DisplayName: Dive mode target depth
    // @Description: Target depth for DIVE mode (positive value, in meters)
    // @Range: 0 100
    // @Units: m
    AP_GROUPINFO("DEPTH", 1, ModeDive, target_depth, 5),

    // @Param: PITCH
    // @DisplayName: Dive mode pitch
    // @Description: Target pitch angle for the initial dive (negative for nose down)
    // @Range: -45 0
    // @Units: deg
    AP_GROUPINFO("PITCH", 2, ModeDive, dive_pitch, -15),

    AP_GROUPEND
};

ModeDive::ModeDive() : Mode() {
    AP_Param::setup_object_defaults(this, var_info);
}

bool ModeDive::_enter() {
    dive_started = false;
    current_dive_state = ACCELERATING;
    return true;
}

void ModeDive::update() 
{
    if (!(plane.current_loc.initialised() && AP::ahrs().home_is_set())) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        return;
    }

    const float depth_target = target_depth; 
    
    if (!dive_started) {
        start_loc = plane.current_loc;
        plane.prev_WP_loc = plane.current_loc;
        plane.next_WP_loc = plane.current_loc;
        
        // Target altitude is calculated as current altitude minus depth target
        plane.next_WP_loc.alt -= depth_target * 100.0;

        plane.crash_state.is_crashed = false;
        plane.set_flight_stage(AP_FixedWing::FlightStage::TAKEOFF); 

        gcs().send_text(MAV_SEVERITY_INFO, "Diving to %.0fm", depth_target);
        dive_started = true;
    }

    // Calculate current depth relative to start location
    float current_depth_cm = start_loc.alt - plane.current_loc.alt;
    float current_speed = ahrs.groundspeed();

    if (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF) {
        
        switch (current_dive_state) {
            case ACCELERATING:
                SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100.0);
                plane.auto_state.takeoff_pitch_cd = 0; 

                // Wait for enough water flow over the fins
                if (current_speed > 1.5) { 
                    current_dive_state = PITCHING_DOWN;
                }
                break;

            case PITCHING_DOWN:
                SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100.0);
                plane.auto_state.takeoff_pitch_cd = dive_pitch * 100; 

                // Pull out 0.5m before target depth
                if (current_depth_cm >= (depth_target * 100) - 50) { 
                    current_dive_state = LEVEL_OUT;
                }
                break;

            case LEVEL_OUT:
                plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
                break;
        }

        // Force roll LEVEL_OUT during dive and override standard pitch logic
        plane.nav_roll_cd = 0; 
        plane.nav_pitch_cd = plane.auto_state.takeoff_pitch_cd;

    } else {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}