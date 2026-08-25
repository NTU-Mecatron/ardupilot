# Modifying ArduPlane to make it compatible for Torpedo-shaped AUV with control surfaces

## Assumptions / Already Available
- **Positioning**: Available via GPS or external positioning (AP_NavEKF3 external positioning)
- **Depth Sensor**: Already in `libraries/AP_Baro/AP_MS5611` (reads depth from pressure)
- **Velocity**: Externally fed into the system
- **Leak Detection**: Already implemented
- **Physics Testing**: External physics backend for SITL

Current assumption: L1 and TECS are still applicable for underwater use case.

## Core Control Comparison: Aircraft vs Torpedo AUV

### Similarities
- Pitch for altitude/depth control
- libraries/AP_Baro already provides depth from pressure sensor following the same sign convention as altitude (negative = deeper)

### Differences

| Aspect | Aircraft | Torpedo AUV |
|--------|----------|-------------|
| Lateral Control | **Roll for turning** | **Yaw for turning, roll must try to be balanced as much as possible** |
| Coordination | Yaw for coordination | Roll less critical / used differently |

---

## Control Flow Impact Analysis

**AUV Modifications Fit Into**:
1. **init()** — Configure barometer to BARO_TYPE_WATER
2. **update_control_mode()** (Fast) → calls **mode->update()** — Add yaw calculation instead of roll
3. **stabilize()** (Fast) — Route yaw instead of roll, keep pitch unchanged
4. **servos_output()** (Fast) — Replace mixing with fin allocation, read pitch/yaw/roll outputs

---

## Implementation strategy

### 1. Configuring and Setting Up

**Files to modify:**
- `ArduPlane/Plane.h` — Add is_auv_mode flag
- `ArduPlane/ArduPlane.cpp` — Initialize barometer for water sensor in `init()`

**Changes needed in Plane.h:**
```cpp
// Add to Plane class
bool is_auv_mode = false;  // Runtime flag to enable AUV control allocation
```

**Configuration in init() or setup:**
```cpp
// In ArduPlane.cpp init() or similar startup location
#ifdef AUV_MODE
if (is_auv_mode) {
    // Configure AP_Baro for water depth measurement (like ArduSub does)
    // Set barometer to BARO_TYPE_WATER (calculates depth instead of altitude)
    barometer.set_type(0, AP_Baro::BARO_TYPE_WATER);  // Set primary barometer as water sensor
}
#endif
```

### 2. Assuming T fins (like an aircraft), test out how original ArduPlane control applies to TorpAUV in terms of diving and maintaining depth

Keep `ArduPlane/mode_takeoff.cpp`; takeoff altitude can be set to negative to make it work for the AUV. By default, after taking off, the AUV will loiter at a distance around the launch point, but we will skip it for now and make it keep go straight. Purpose is to test out the control flow and see if it can maintain depth.

No waypoint/yaw navigation here as it requires changing from roll-to-turn to yaw-to-turn.

### 3. Yaw for Waypoint Navigation (Instead of Roll)

**Key Insight**: Aircraft rolls to turn, AUV yaws to turn (like a submarine or ship). Replace the roll command generation in mode updates with yaw command generation.

**Control Flow Point**: The fast loop calls `update_control_mode()` → which calls `mode->update()` → which calls navigation functions.

#### 3.1 Add Navigation Helper Function
**File**: [ArduPlane/navigation.cpp](ArduPlane/navigation.cpp) — NEW FUNCTION

Add `calc_nav_yaw()` function to complement existing `calc_nav_roll()`:
```cpp
void Plane::calc_nav_yaw(void) {
#ifdef AUV_MODE
    if (is_auv_mode) {
        // L1 controller outputs target_bearing_cd()
        // We use this directly for yaw instead of converting to roll
        float bearing_error_cd = wrap_180_cd(
            nav_controller->target_bearing_cd() - ahrs.yaw_sensor
        );
        nav_yaw_cd = bearing_error_cd;  // Desired yaw (centidegrees)
        return;
    }
#endif
    calc_nav_roll();  // Original aircraft roll calculation
}
```
**Note**: No L1 controller changes needed — L1 already calculates bearing/target_bearing_cd()

#### 3.2 Modify Mode Update Functions
**Files**: `ArduPlane/mode_auto.cpp`, `ArduPlane/mode_guided.cpp`, `ArduPlane/mode_loiter.cpp`, etc.

These modes currently call: `calc_nav_roll()`, `calc_nav_pitch()`, `calc_throttle()`

**For AUV mode**, change to call `calc_nav_yaw()` instead:
```cpp
void ModeAuto::update() {
#ifdef AUV_MODE
    if (plane.is_auv_mode) {
        plane.calc_nav_yaw();         // Yaw for waypoint heading (replaces calc_nav_roll)
        plane.calc_nav_pitch();       // Pitch for depth (unchanged semantically)
        plane.calc_throttle();        // Throttle from TECS (unchanged)
    } else
#endif
    {
        plane.calc_nav_roll();        // Roll for waypoint heading (aircraft)
        plane.calc_nav_pitch();       // Pitch for altitude (aircraft)
        plane.calc_throttle();        // Throttle (unchanged)
    }
}
```

**Result**: `nav_yaw_cd` is set instead of `nav_roll_cd`. The rest of the control flow (TECS, throttle) is unaffected.

#### 3.3 Modify Stabilization Loop
**File**: [ArduPlane/Attitude.cpp](ArduPlane/Attitude.cpp) — The `stabilize()` function (Fast, ~400Hz)

**Current aircraft flow**: `stabilize_roll()`, `stabilize_pitch()`, `stabilize_yaw()`

**New AUV flow**: Use `nav_yaw_cd` from step 2.1 to command yaw instead of roll

Modify `stabilize()`:
```cpp
void Plane::stabilize() {
#ifdef AUV_MODE
    if (is_auv_mode) {
        // AUV attitude control: yaw for heading, pitch for depth
        stabilize_yaw_heading();     // Yaw control (reads nav_yaw_cd, outputs to rudder)
        stabilize_pitch();           // Pitch control (unchanged)
        // Note: stabilize_roll() not called in AUV mode
    } else
#endif
    {
        // Aircraft attitude control: roll for heading
        stabilize_roll();            // Roll (reads nav_roll_cd)
        stabilize_pitch();           // Pitch (unchanged)
        stabilize_yaw();             // Yaw coordination (uses roll rate for rudder)
    }
}
```

#### 3.4 Add Yaw Heading Stabilization Controller
**File**: [ArduPlane/Attitude.cpp](ArduPlane/Attitude.cpp) — NEW FUNCTION (Fast, ~400Hz)

This parallels `stabilize_roll()` but uses yaw instead:
```cpp
void Plane::stabilize_yaw_heading(void) {
#ifdef AUV_MODE
    // Yaw error from desired heading (nav_yaw_cd set in calc_nav_yaw)
    float yaw_error_cd = wrap_180_cd(nav_yaw_cd - ahrs.yaw_sensor);
    
    // Use yawController (existing PID) to get rudder command
    float speed_scaler = get_speed_scaler();
    float yaw_out = yawController.get_servo_out(
        yaw_error_cd,           // Desired yaw angle error
        speed_scaler,           // Scale gains by airspeed (water speed)
        false                   // use_rate = false (use angle control, not rate)
    );
    
    // Output directly to rudder servo
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, yaw_out);
#endif
}
```

**Note**: This reuses the existing `yawController` PID that was designed for coordinated turns. For AUV, it now provides primary heading control instead of coordination.

---

### 4. Moving away from T-fin assumption to Arbitrary Fin Allocation System

**Key Insight**: Fins can be in various configurations (+, X, H, etc.). Need flexible allocation matrix.

**Control Flow Point**: `set_servos()` (Fast ~400Hz) → `servos_output()` — This is where control surface mixing happens.

#### 4.1 Create Fin Allocation System
**New files**: `ArduPlane/fin_allocation.cpp` and `fin_allocation.h`

Class structure:
```cpp
class FinAllocation {
public:
    struct FinConfig {
        uint8_t servo_function;           // SRV_Channel function (e.g., k_fin_1)
        float pitch_effectiveness;        // -1.0 to 1.0 (how much pitch affects this fin)
        float yaw_effectiveness;          // -1.0 to 1.0 (how much yaw affects this fin)
        float roll_effectiveness;         // -1.0 to 1.0 (may be unused for AUV)
    };
    
    void init();
    void allocate_fins(float pitch_cmd_cd, float yaw_cmd_cd, float roll_cmd_cd);
    
    FinConfig fins[8];  // Up to 8 fins
    uint8_t num_fins;
    
private:
    void load_config();  // Load fin config from parameters
};
```

#### 4.2 Implement Simple Linear Allocation
**File**: `ArduPlane/fin_allocation.cpp`

Initial approach (no optimization):
```cpp
void FinAllocation::allocate_fins(float pitch_cd, float yaw_cd, float roll_cd) {
    // Linear allocation: fin_deflection = weighted sum of control inputs
    
    for (uint8_t i = 0; i < num_fins; i++) {
        if (fins[i].servo_function == 0) continue;  // Skip unconfigured fins
        
        float fin_deflection_cd = 
            pitch_cd * fins[i].pitch_effectiveness +
            yaw_cd * fins[i].yaw_effectiveness +
            roll_cd * fins[i].roll_effectiveness;
        
        // Constrain to servo range
        fin_deflection_cd = constrain_int32(fin_deflection_cd, -4500, 4500);
        
        // Output to servo channel
        SRV_Channels::set_output_scaled(fins[i].servo_function, fin_deflection_cd);
    }
}
```

#### 4.3 Integrate with Servo Output Pipeline
**File**: [ArduPlane/servos.cpp](ArduPlane/servos.cpp) — `servos_output()` function (Fast ~400Hz)

**Current flow**: Reads elevator/aileron/rudder from SRV_Channels, applies mixing (elevon, v-tail, etc.), outputs to servos

**New AUV flow**: Same inputs, but route through fin allocation instead of traditional mixing

```cpp
void Plane::servos_output(void) {
    SRV_Channels::cork();  // Batch updates
    
#ifdef AUV_MODE
    if (is_auv_mode) {
        // Read control outputs set by stabilize()
        // These are in the standard aircraft channel format (elevator, aileron, rudder)
        float pitch_cmd = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
        float yaw_cmd = SRV_Channels::get_output_scaled(SRV_Channel::k_rudder);
        float roll_cmd = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
        
        // Clear outputs (they'll be set by fin allocation)
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, 0);
        
        // Allocate to arbitrary fin configuration
        g2.fin_allocation.allocate_fins(pitch_cmd, yaw_cmd, roll_cmd);
    } else
#endif
    {
        // Original aircraft servo mixing
        servos_twin_engine_mix();
        channel_function_mixer(SRV_Channel::k_aileron, SRV_Channel::k_elevator, 
                              SRV_Channel::k_elevon_left, SRV_Channel::k_elevon_right);
        // ... other mixing (v-tail, flaps, spoilers, etc.)
    }
    
    // Convert to PWM and output (unchanged)
    SRV_Channels::calc_pwm();
    SRV_Channels::output_ch_all();
    SRV_Channels::push();
}
```

#### 4.4 Add Fin Configuration Parameters
**File**: `ArduPlane/Parameters.h` — Add parameter group for fin configuration

Define parameters for each of up to 8 fins:
```cpp
// Example for FIN1 (repeat for FIN2–FIN8)

// @Param: FIN1_SRV
// @DisplayName: Fin 1 Servo Function
// @Description: Servo output number to use for Fin 1
// @Values: 0:None,1-16:Channel1-16
AP_GROUPINFO("FIN1_SRV", XX, Plane, g2.fin_alloc.fins[0].servo_function, 0),

// @Param: FIN1_PITCH
// @DisplayName: Fin 1 Pitch Effectiveness
// @Description: How much this fin responds to pitch commands
// @Range: -1.0 1.0
// @Increment: 0.1
AP_GROUPINFO("FIN1_PITCH", XX, Plane, g2.fin_alloc.fins[0].pitch_effectiveness, 0.0),

// @Param: FIN1_YAW
// @DisplayName: Fin 1 Yaw Effectiveness
// @Description: How much this fin responds to yaw commands
// @Range: -1.0 1.0
// @Increment: 0.1
AP_GROUPINFO("FIN1_YAW", XX, Plane, g2.fin_alloc.fins[0].yaw_effectiveness, 0.0),

// @Param: FIN1_ROLL
// @DisplayName: Fin 1 Roll Effectiveness
// @Description: How much this fin responds to roll commands (usually 0 for AUV)
// @Range: -1.0 1.0
// @Increment: 0.1
AP_GROUPINFO("FIN1_ROLL", XX, Plane, g2.fin_alloc.fins[0].roll_effectiveness, 0.0),
```

#### 4.5 Define Fin Servo Channel Types
**File**: `libraries/AP_HAL/AP_HAL.h` or `libraries/AP_SRV_Channel/AP_SRV_Channel.h` (SRV_Channel definitions)

Add new channel types for fins:
```cpp
enum SRV_Channel::Aux_servo_function {
    // ... existing functions ...
    k_fin_1 = XX,    // Fin 1
    k_fin_2 = XX,    // Fin 2
    // ... etc up to k_fin_8
};
```

This allows parameters to reference fin channels consistently.

---

## Control Flow Diagram: AUV vs Aircraft

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
│                   calc_nav_pitch() for depth          │
│                                                         │
│  2. stabilize()                                        │
│     ├─► [Aircraft] stabilize_roll() ──► aileron      │
│     │              stabilize_pitch() ─► elevator      │
│     │              stabilize_yaw() ───► rudder        │
│     │                                                  │
│     └─► [AUV] stabilize_yaw_heading() ──► rudder     │
│               stabilize_pitch() ────────► elevator    │
│                                                         │
│  3. set_servos()                                       │
│     └─► servos_output()                               │
│         ├─► [Aircraft] Direct mixing (elevon, v-tail) │
│         │                                              │
│         └─► [AUV] fin_allocation.allocate_fins()     │
│                   (pitch, yaw, roll) ──► fins[1-8]    │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

---

## Future Enhancements

These are deferred beyond Phase 1 but documented for roadmap:

### Advanced Fin Allocation (Optimization Solver)
- Replace linear allocation with constrained least-squares optimization
- Minimize fin deflections while achieving target attitude
- Handle fin saturation gracefully (saturate least-important axis first)

### Adaptive Control for Marine Dynamics
- Estimate dynamic pressure from water speed
- Auto-tune TECS and attitude controller gains based on speed

### Thruster Vectoring (if applicable)
- Support thrusters with independently controllable angles
- Allocate thrust vector to complement fin control