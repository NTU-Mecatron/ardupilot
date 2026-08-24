# Simplified Torpedo AUV Implementation Plan for ArduPlane

## Overview
This document outlines the focused code changes needed to adapt ArduPlane for a torpedo-shaped AUV that uses control surfaces (fins) for maneuvering.

## Assumptions / Already Available
- **Positioning**: Available via GPS or external positioning (AP_NavEKF3 external positioning)
- **Depth Sensor**: Already in `libraries/AP_Baro/AP_MS5611` (reads depth from pressure)
- **Frame Convention**: Forward-Right-Down (depth is positive downward)
- **Velocity**: Externally fed into the system
- **Leak Detection**: Already implemented
- **Physics Testing**: External physics backend for SITL

## Core Control Differences: Aircraft vs Torpedo AUV

| Aspect | Aircraft | Torpedo AUV |
|--------|----------|-------------|
| Vertical Control | Pitch for altitude | Pitch for depth (positive down) |
| Lateral Control | **Roll for turning** | **Yaw for turning** |
| Coordination | Yaw for coordination | Roll less critical / used differently |
| Reference | Altitude (barometer) | Depth (pressure sensor) |

## Critical Control Architecture Changes

### 1. Depth Control via Pitch (Instead of Altitude)

**Key Insight**: Aircraft pitches to control altitude; AUV pitches to control depth.

#### 1.1 Depth Reference System
**Files to modify:**
- `altitude.cpp` - Adapt for depth (or create `depth.cpp`)
- `Plane.h` - Add depth tracking variables

**Changes needed:**
```cpp
// In Plane.h
#ifdef AUV_MODE
    float current_depth_m;              // From barometer (positive down)
    float target_depth_m;               // Desired depth
    float depth_error_m;                // target - current
#else
    float relative_altitude;
    // ... existing altitude code
#endif

// In altitude.cpp or new depth.cpp
void Plane::update_depth(void) {
#ifdef AUV_MODE
    // Read depth from MS5611 pressure sensor
    current_depth_m = barometer.get_altitude() * -1.0f;  // Negative altitude = positive depth
    
    // Calculate depth error for controller
    depth_error_m = target_depth_m - current_depth_m;
    
    // Log depth
    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Depth();
    }
#else
    // Original altitude update
#endif
}
```

#### 1.2 Pitch Controller for Depth
**Files to modify:**
- `Attitude.cpp` - Modify pitch calculation
- `libraries/AP_TECS/` or create simple depth controller

**WHERE to implement:**
```cpp
// In Attitude.cpp - calc_nav_pitch() or new calc_depth_pitch()
void Plane::calc_depth_pitch(void) {
#ifdef AUV_MODE
    // Simple depth-to-pitch controller
    // Positive depth error (too shallow) -> positive pitch (nose down)
    // Negative depth error (too deep) -> negative pitch (nose up)
    
    float depth_error = target_depth_m - current_depth_m;
    
    // PID control for depth via pitch
    // Can reuse existing pitch controller or create depth_controller
    nav_pitch_cd = depth_controller.get_pitch_for_depth(
        depth_error,
        get_speed_scaler()
    );
    
    // Constrain pitch based on limits
    nav_pitch_cd = constrain_int32(nav_pitch_cd, 
                                    pitch_limit_min*100, 
                                    aparm.pitch_limit_max.get()*100);
#else
    // Original pitch calculation for altitude
#endif
}
```

**Integration point in mode update:**
```cpp
// In mode_auto.cpp or other modes
void ModeAuto::update() {
#ifdef AUV_MODE
    plane.calc_nav_yaw();        // Yaw for waypoint tracking (NEW!)
    plane.calc_depth_pitch();    // Pitch for depth control
    plane.calc_throttle();       // Thrust control
#else
    plane.calc_nav_roll();       // Roll for waypoint tracking
    plane.calc_nav_pitch();      // Pitch for altitude
    plane.calc_throttle();
#endif
}
```

### 2. Yaw for Waypoint Navigation (Instead of Roll)

**Key Insight**: Aircraft rolls to turn, AUV yaws to turn (like a submarine or ship).

#### 2.1 Waypoint Tracking via Yaw
**Files to modify:**
- `navigation.cpp` - Calculate desired yaw instead of roll
- `Attitude.cpp` - Add yaw tracking controller
- `libraries/AP_L1_Control/` - Modify to output yaw instead of roll

**WHERE to implement:**
```cpp
// In navigation.cpp - New function or modify calc_nav_roll
void Plane::calc_nav_yaw(void) {
#ifdef AUV_MODE
    // Use L1 controller but output yaw instead of roll
    // L1 gives us lateral acceleration command
    float lateral_accel = nav_controller->nav_roll_cd() / 100.0f;  // Reuse L1 output
    
    // Convert lateral acceleration to yaw rate
    // For AUV: yaw rate = lateral_accel / velocity
    float velocity = ahrs.groundspeed();
    if (velocity > 0.5f) {
        float desired_yaw_rate = lateral_accel / velocity;
        nav_yaw_rate = constrain_float(desired_yaw_rate, -45, 45);  // deg/s
    }
    
    // Or directly calculate bearing error
    float bearing_error_cd = wrap_180_cd(
        nav_controller->target_bearing_cd() - ahrs.yaw_sensor
    );
    
    // P controller for yaw
    nav_yaw_cd = bearing_error_cd;  // Will be fed to yaw controller
#else
    // Original roll calculation
    calc_nav_roll();
#endif
}
```

#### 2.2 Yaw Stabilization Controller
**Files to modify:**
- `Attitude.cpp` - Create stabilize_yaw_heading() function

**WHERE to implement:**
```cpp
// In Attitude.cpp - in stabilize() function
void Plane::stabilize() {
#ifdef AUV_MODE
    // AUV control allocation
    stabilize_pitch();  // For depth control
    stabilize_yaw_heading();  // For waypoint tracking (NEW!)
    stabilize_roll();   // For coordinated turns or may be unused
#else
    // Aircraft control
    stabilize_roll();   // For waypoint tracking
    stabilize_pitch();  // For altitude control  
    stabilize_yaw();    // For coordination
#endif
}

void Plane::stabilize_yaw_heading(void) {
#ifdef AUV_MODE
    // Track desired heading via yaw controller
    float yaw_error_cd = wrap_180_cd(nav_yaw_cd - ahrs.yaw_sensor);
    
    const float speed_scaler = get_speed_scaler();
    float yaw_out = yawController.get_servo_out(yaw_error_cd, speed_scaler, false);
    
    // Output to rudder/yaw control
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, yaw_out);
#endif
}
```

#### 2.3 L1 Controller Adaptation
**Files to modify:**
- `libraries/AP_L1_Control/AP_L1_Control.cpp` - Add yaw output mode

**Conceptual change:**
```cpp
// In AP_L1_Control
#ifdef AUV_MODE
    // Instead of returning roll angle, return yaw/heading command
    int32_t nav_bearing_cd() const {
        // Return desired bearing for yaw control
        return wrap_180_cd(_nav_bearing);
    }
#else
    int32_t nav_roll_cd() const {
        // Return roll angle for aircraft
    }
#endif
```

### 3. Arbitrary Fin Allocation System

**Key Insight**: Fins can be in various configurations (X, +, H, etc.). Need flexible allocation matrix.

#### 3.1 Fin Allocation Architecture
**WHERE to implement:**
- **New file**: `ArduPlane/fin_allocation.cpp` and `fin_allocation.h`
- **Called from**: `servos.cpp` in `servos_output()` function

**Allocation Matrix Concept:**
```cpp
// in fin_allocation.h
class FinAllocation {
public:
    void init();
    void allocate_fins(float pitch_cmd, float yaw_cmd, float roll_cmd);
    
    // Allocation matrix: maps [pitch, yaw, roll] to fin deflections
    // Each fin has effectiveness in each axis
    struct FinConfig {
        uint8_t servo_function;     // SRV_Channel function
        float pitch_effectiveness;  // -1.0 to 1.0
        float yaw_effectiveness;    // -1.0 to 1.0
        float roll_effectiveness;   // -1.0 to 1.0
    };
    
    FinConfig fins[8];  // Up to 8 fins
    uint8_t num_fins;
    
private:
    void solve_allocation();  // Allocate commands to fins (future: optimization)
};
```

**Implementation in fin_allocation.cpp:**
```cpp
void FinAllocation::allocate_fins(float pitch_cmd, float yaw_cmd, float roll_cmd) {
    // For now: simple linear allocation (future: constrained optimization)
    
    for (uint8_t i = 0; i < num_fins; i++) {
        float fin_deflection = 
            pitch_cmd * fins[i].pitch_effectiveness +
            yaw_cmd * fins[i].yaw_effectiveness +
            roll_cmd * fins[i].roll_effectiveness;
        
        // Constrain and output
        fin_deflection = constrain_float(fin_deflection, -4500, 4500);
        SRV_Channels::set_output_scaled(fins[i].servo_function, fin_deflection);
    }
}
```

#### 3.2 Integration with Servo Output
**Files to modify:**
- `servos.cpp` - Call fin allocation instead of direct mixing

**WHERE to call:**
```cpp
// In servos.cpp - servos_output() function
void Plane::servos_output(void) {
    SRV_Channels::cork();

#ifdef AUV_MODE
    // Get control commands from stabilization
    float pitch_cmd = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
    float yaw_cmd = SRV_Channels::get_output_scaled(SRV_Channel::k_rudder);
    float roll_cmd = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
    
    // Allocate to arbitrary fin configuration
    g2.fin_allocation.allocate_fins(pitch_cmd, yaw_cmd, roll_cmd);
    
#else
    // Original aircraft mixing
    servos_twin_engine_mix();
    channel_function_mixer(SRV_Channel::k_aileron, SRV_Channel::k_elevator, 
                          SRV_Channel::k_elevon_left, SRV_Channel::k_elevon_right);
    // ... etc
#endif

    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();
    SRV_Channels::push();
}
```

#### 3.3 Fin Configuration Parameters
**Files to modify:**
- `Parameters.h` - Add fin configuration parameters

**Parameter structure:**
```cpp
// Summary: Critical Code Integration Points

### Control Flow for AUV vs Aircraft

```
┌─────────────────────────────────────────────────────────┐
│                    FAST LOOP (~400Hz)                   │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  1. update_control_mode()                              │
│     └─► mode->update()                                 │
│         ├─► [Aircraft] calc_nav_roll() for waypoint   │
│         │               calc_nav_pitch() for altitude  │
│         │                                              │
│         └─► [AUV] calc_nav_yaw() for waypoint        │
│                   calc_depth_pitch() for depth        │
│                                                         │
│  2. stabilize()                                        │
│     ├─► [Aircraft] stabilize_roll() ──► aileron      │
│     │              stabilize_pitch() ─► elevator      │
│     │              stabilize_yaw() ───► rudder        │
│     │                                                  │
│     └─► [AUV] stabilize_yaw_heading() ──► yaw_cmd   │
│               stabilize_pitch() ────────► pitch_cmd   │
│               stabilize_roll() ─────────► roll_cmd    │
│                                                         │
│  3. set_servos()                                       │
│     └─► servos_output()                               │
│         ├─► [Aircraft] Direct mixing (elevon, vtail) │
│         │                                              │
│         └─► [AUV] fin_allocation.allocate_fins()     │
│                   (pitch, yaw, roll) ──► fins[1-8]    │
│                                                         │
└─────────────────────────────────────────────────────────┘   set_throttle();  // Original function
#endif
}
```

#### 4.2 Control Surface Mapping
**Files to modify:**
- `servos.cpp` - Map to fin configuration
- `Parameters.h` - Add fin configuration parameters

**Typical torpedo configuration:**
```cpp
// Four fins in + configuration
// Top fin (elevator)
// Bottom fin (elevator) 
// Left fin (rudder/roll)
// Right fin (rudder/roll)

void Plane::map_torpedo_fins(void) {
    // Get stabilized outputs
    float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
    float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
    float rudder = SRV_Channels::get_output_scaled(SRV_Channel::k_rudder);
    
    // Map to four fins
    float top_fin = elevator;
    float bottom_fin = elevator;
    float left_fin = aileron + rudder * g.rudder_mix;
    float right_fin = -aileron + rudder * g.rudder_mix;
    
    SRV_Channels::set_output_scaled(SRV_Channel::k_fin_top, top_fin);
    SRV_Channels::set_output_scaled(SRV_Channel::k_fin_bottom, bottom_fin);
    SRV_Channels::set_output_scaled(SRV_Channel::k_fin_left, left_fin);
    SRV_Channels::set_output_scaled(SRV_Channel::k_fin_right, right_fin);
}
```

### Key Files to Create/Modify

#### New Files:
1. **`ArduPlane/fin_allocation.cpp/h`** - Fin allocation system
2. **`ArduPlane/depth.cpp`** - Depth control (or adapt altitude.cpp)

#### Modified Files:
1. **`ArduPlane/Attitude.cpp`**
   - Add `calc_depth_pitch()` function
   - Add `stabilize_yaw_heading()` function  
   - Modify `stabilize()` to call AUV-specific functions

2. **`ArduPlane/navigation.cpp`**
   - Add `calc_nav_yaw()` function for waypoint tracking

3. **`ArduPlane/servos.cpp`**
   - Modify `servos_output()` to call fin allocation

4. **`ArduPlane/mode_auto.cpp`** (and other modes)
   - Change control allocation from roll/pitch to yaw/pitch

5. **`ArduPlane/Parameters.h`**
   - Add fin configuration parameters

6. **`ArduPlane/config.h`**
   - Add AUV_MODE compilation flag

7. **`libraries/AP_L1_Control/`** (optional)
   - Adapt to output heading/bearing instead of roll

## Files Summary

### Files to Create (New)
- `libraries/AP_DepthSensor/`
- `libraries/AP_UECS/`
- `libraries/AP_LeakDetector/`
- `libraries/AP_DVL/`
- `libraries/AP_Ballast/`
- `ArduPlane/mode_depth_hold.cpp`
- `ArduPlane/mode_surface.cpp`
- `ArduPlane/mode_dive.cpp`
- `ArduPlane/emergency_surface.cpp`
- `ArduPlane/dead_reckoning.cpp`

### Files to Heavily Modify (Conditional Compilation)
- `ArduPlane/ArduPlane.cpp` - Scheduler
- `ArduPlane/Plane.h` - Main class
- `ArduPlane/Attitude.cpp` - Controllers
- `ArduPlane/navigation.cpp` - Navigation
- `ArduPlane/servos.cpp` - Output
- `ArduPlane/Parameters.h/cpp` - Parameters
- `ArduPlane/failsafe.cpp` - Safety
- `ArduPlane/mode_auto.cpp` - AUTO mode

### Files to Lightly Modify
- All `mode_*.cpp` files - Add AUV checks
- `ArduPlane/config.h` - Feature flags
- `ArduPlane/defines.h` - AUV enums
- `wscript` - Build configuration

## Estimated Effort

- **Total Development**: 12-16 weeks for core functionality
- **Testing & Validation**: 8-12 weeks
- **Documentation**: 2-3 weeks
- **Total Project**: 22-31 weeks (5.5-7.5 months)

This assumes:
- Experienced ArduPilot developer
- Access to simulation environment
- Access to test hardware
- No major architectural changes required
Implementation Checklist

### Phase 1: Depth Control (1-2 weeks)
- [ ] Add depth tracking variables to `Plane.h`
- [ ] Create/modify `depth.cpp` to read from MS5611 (positive depth = down)
- [ ] Implement `calc_depth_pitch()` in `Attitude.cpp`
- [ ] Add simple depth PID controller
- [ ] Test depth hold in SITL

### Phase 2: Yaw Navigation (1-2 weeks)  
- [ ] Implement `calc_nav_yaw()` in `navigation.cpp`
- [ ] Implement `stabilize_yaw_heading()` in `Attitude.cpp`
- [ ] Modify L1 controller or create bearing-based navigation
- [ ] Modify mode `update()` functions to call yaw instead of roll
- [ ] Test waypoint following in SITL

### Phase 3: Fin Allocation (2-3 weeks)
- [ ] Create `fin_allocation.cpp/h` files
- [ ] Implement allocation matrix and parameters
- [ ] Add fin configuration parameters to `Parameters.h`
- [ ] Modify `servos_output()` to call fin allocation
- [ ] Test with different fin configurations (+, X, etc.)
- [ ] Add parameter loading/validation

### Phase 4: Integration & Testing (1-2 weeks)
- [ ] Ensure failsafe behavior (disarm if no positioning)
- [ ] Test full AUTO mission with waypoints and depth changes
- [ ] Verify all control modes work with AUV control allocation
- [ ] Document parameter configuration guide
- [ ] Create example configurations for common fin layouts

## Estimated Effort

- **Phase 1 (Depth)**: 1-2 weeks
- **Phase 2 (Yaw Nav)**: 1-2 weeks  
- **Phase 3 (Fin Alloc)**: 2-3 weeks
- **Phase 4 (Integration)**: 1-2 weeks
- **Total**: 5-9 weeks for core AUV control

This is significantly simplified from the original 22-31 week estimate by:
- Using existing positioning system (no dead reckoning)
- Using existing sensors (MS5611, leak detector)
- Using external velocity (no water speed sensor)
- Using external physics backend (no SITL modifications)
- Focusing only on control allocation changes