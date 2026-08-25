# Simplified Torpedo AUV Implementation Plan for ArduPlane

## Overview
This document outlines the focused code changes needed to adapt ArduPlane for a torpedo-shaped AUV that uses control surfaces (fins) for maneuvering.

## Assumptions / Already Available
- **Positioning**: Available via GPS or external positioning (AP_NavEKF3 external positioning)
- **Depth Sensor**: Already in `libraries/AP_Baro/AP_MS5611` (reads depth from pressure)
- **Velocity**: Externally fed into the system
- **Leak Detection**: Already implemented
- **Physics Testing**: External physics backend for SITL

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

Based on ArduPlane's architecture (from code_structure.md):
- **Fast Loop (400Hz)**: `ahrs_update` → `update_control_mode` → `stabilize` → `set_servos`
- **Medium Rate (10Hz)**: `navigate` (waypoint bearing), `update_alt` (altitude/TECS), `calc_airspeed_errors`
- **Servo Output Flow**: `set_servos` → `servos_output()` → mixing → PWM output

**AUV Modifications Fit Into**:
1. **init()** — Configure barometer to BARO_TYPE_WATER
2. **update_control_mode()** (Fast) → calls **mode->update()** — Add yaw calculation instead of roll
3. **stabilize()** (Fast) — Route yaw instead of roll, keep pitch unchanged
4. **servos_output()** (Fast) — Replace mixing with fin allocation, read pitch/yaw/roll outputs

---

## Control Architecture Plans

### 1. Depth Control via Pitch (Instead of Altitude)

**Key Insight**: Aircraft pitches to control altitude; AUV pitches to control depth. AP_Baro already calculates depth for water sensors—just use it directly, like ArduSub does.

#### 1.1 Add AUV Mode Flag and Configure Barometer
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

#### 1.2 Use Barometer Depth Directly in TECS/Control Loop
**No special handling needed**: Follow ArduSub's pattern.

- `barometer.get_altitude()` returns altitude/depth (meters)
  - Positive values above water (rarely used for AUV)
  - Negative values underwater: -5m means 5 meters deep
- TECS will use this for depth control (same as altitude control for aircraft)
- Altitude/depth coordinate frame is separate from body frame (FRD)
- Just use the values as-is—AP_Baro already handles sign convention correctly

#### 1.3 Verify No Modifications Needed Downstream
- ✓ `relative_target_altitude_cm()` — Works as-is (now represents depth target, negative = deeper)
- ✓ TECS controller — Works unchanged (operates on depth error with same control logic)
- ✓ Flight modes (AUTO, GUIDED, etc.) — Work unchanged (use relative_altitude for depth targets)
- ✓ Mission waypoints — Altitude field becomes depth field (negative = deeper underwater)
  - Example: altitude = -5.0m in mission = dive to 5 meters deep
- ✓ AP_Baro density parameter — User can tune `BARO_SPEC_GRAV` in GCS for water type
- **Note**: TECS may need gain tuning for marine dynamics (slower response). Configurable via TECS_* parameters.

---

### 2. Yaw for Waypoint Navigation (Instead of Roll)

**Key Insight**: Aircraft rolls to turn, AUV yaws to turn (like a submarine or ship). Replace the roll command generation in mode updates with yaw command generation.

**Control Flow Point**: The fast loop calls `update_control_mode()` → which calls `mode->update()` → which calls navigation functions.

#### 2.1 Add Navigation Helper Function
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

#### 2.2 Modify Mode Update Functions
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

#### 2.3 Modify Stabilization Loop
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

#### 2.4 Add Yaw Heading Stabilization Controller
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

### 3. Arbitrary Fin Allocation System

**Key Insight**: Fins can be in various configurations (+, X, H, etc.). Need flexible allocation matrix.

**Control Flow Point**: `set_servos()` (Fast ~400Hz) → `servos_output()` — This is where control surface mixing happens.

#### 3.1 Create Fin Allocation System
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

#### 3.2 Implement Simple Linear Allocation
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

#### 3.3 Integrate with Servo Output Pipeline
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

#### 3.4 Add Fin Configuration Parameters
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

#### 3.5 Define Fin Servo Channel Types
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

### 4.2 Failsafe Behavior (Safety-Critical)
**File**: [ArduPlane/failsafe.cpp](ArduPlane/failsafe.cpp)

Add depth/pressure-based failsafe:
```cpp
void Plane::failsafe_check(void) {
#ifdef AUV_MODE
    if (is_auv_mode) {
        // Check if depth exceeds safe limit
        // Note: altitude is negative when underwater (more negative = deeper)
        float current_depth = -barometer.get_altitude();  // Convert to positive depth in meters
        float max_depth_m = aparm.max_depth_m.get();
        
        if (current_depth > max_depth_m) {
            // Exceed maximum depth - surface immediately
            set_mode(MODE_RTL);  // Or MODE_FLOAT (stay at surface)
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Max depth exceeded - surfacing");
        }
        
        // Check for pressure sensor failure (depth suddenly increases unexpectedly)
        static uint32_t last_depth_check_ms = 0;
        static float min_depth_seen = FLT_MAX;
        
        if (millis() - last_depth_check_ms > 1000) {
            if (current_depth < min_depth_seen) {
                // Depth is decreasing (altitude becoming less negative = going shallower unexpectedly)
                // Could indicate sensor failure
                min_depth_seen = current_depth;
            }
            last_depth_check_ms = millis();
        }
    }
#endif
    
    // Original aircraft failsafe checks
    battery_failsafe();
    gps_failsafe();
}
```

### 4.3 TECS Tuning for Marine Dynamics
**Note**: TECS may need parameter retuning for AUV response characteristics

Typical adjustments:
- `TECS_SPDWEIGHT` - Higher value (0.8–1.0) for AUV (thrust more important than pitch)
- `TECS_PTCH_FF` - May need reduction (less pitch authority in water)
- `TECS_TIME_CONST` - May need increase (slower marine dynamics)

### 4.4 Verification Checklist
- [ ] Depth is read correctly from barometer (sign check: negative = deeper, 0 = surface)
- [ ] Pitch command generates down-bow in water (elevator command negative → pitch down → deeper)
- [ ] Yaw navigation follows waypoint bearing
- [ ] Fin allocation outputs to correct channels in configured geometry
- [ ] TECS maintains depth at setpoint (±0.5m tolerance)
- [ ] Failsafe triggers at maximum depth and RTL
- [ ] Log shows correct depth tracking and fin deflections

---

## Phase 5: Future Enhancements (Phase 2+)

These are deferred beyond Phase 1 but documented for roadmap:

### 5.1 Advanced Fin Allocation (Optimization Solver)
- Replace linear allocation with constrained least-squares optimization
- Minimize fin deflections while achieving target attitude
- Handle fin saturation gracefully (saturate least-important axis first)
- **Estimated effort**: 2–3 weeks

### 5.2 Buoyancy Compensation
- Trim depth at zero throttle (neutral buoyancy setpoint)
- Adapt pitch trim to water density changes (salinity variation)
- **Estimated effort**: 1 week

### 5.3 Adaptive Control for Marine Dynamics
- Estimate dynamic pressure from water speed
- Auto-tune TECS and attitude controller gains based on speed
- **Estimated effort**: 2 weeks

### 5.4 Thruster Vectoring (if applicable)
- Support thrusters with independently controllable angles
- Allocate thrust vector to complement fin control
- **Estimated effort**: 2–3 weeks

---

## File Summary

| File | Purpose | Type | Effort |
|------|---------|------|--------|
| Plane.h | Add is_auv_mode flag | Modify | Low |
| ArduPlane.cpp | Configure barometer for water sensor in init() | Modify | Low |
| navigation.cpp | Add calc_nav_yaw() function | Modify | Medium |
| Attitude.cpp | Add stabilize_yaw_heading(), modify stabilize() | Modify | Medium |
| mode_auto.cpp | Call calc_nav_yaw() instead of calc_nav_roll() in AUV mode | Modify | Low |
| mode_guided.cpp, mode_loiter.cpp, etc. | Same as mode_auto.cpp | Modify | Low |
| servos.cpp | Route through fin_allocation.allocate_fins() in AUV mode | Modify | Medium |
| fin_allocation.cpp, fin_allocation.h | NEW: Implement FinAllocation class and linear allocation | Create | Medium |
| Parameters.h | Add fin configuration parameters (FIN1_SRV–FIN8_ROLL) | Modify | Low |
| failsafe.cpp | Add depth limit and sensor failure checks | Modify | Low |
| AP_SRV_Channel.h | Add k_fin_1 through k_fin_8 channel types | Modify | Low |

**Removed from plan (already built into AP_Baro):**
- ✓ barometer_to_depth() function — Not needed; AP_Baro calculates depth when type=BARO_TYPE_WATER
- ✓ water_density parameter — Use AP_Baro's existing BARO_SPEC_GRAV parameter instead
- ✓ altitude.cpp modifications — No custom depth conversion function needed
- ✓ sink_rate negation — AP_Baro already returns correct sign; no frame conversion needed

---

## Design Rationale

**Why nav_yaw_cd instead of modifying L1?**
- L1 already outputs correct bearing/lateral acceleration
- AUV just interprets it as heading command instead of roll command
- L1 optimization logic unchanged
- Easier to test and debug (simpler per-layer changes)

**Why linear fin allocation initially?**
- Fast to implement (1–2 days)
- Sufficient for most AUV configurations (especially + and X)
- Optimization solver deferred to Phase 2 (when we understand failure modes)
- Easy to validate (can verify analytically)

**Why conditional compilation (#ifdef AUV_MODE)?**
- Aircraft builds unaffected (zero overhead)
- Clear separation of concerns (AUV vs. aircraft logic)
- Easy to disable if issues arise
- Future: could be runtime flag (is_auv_mode) without #ifdefs

---

## Risk Mitigation

| Risk | Mitigation |
|------|-----------|
| TECS controller behaves unexpectedly in water | Start with conservative gains; test in SITL extensively |
| Barometer failure at depth | Add failsafe check for sudden depth changes; redundant sensor fallback |
| Fin allocation produces unstable fin deflections | Start with small effectiveness values; validate mathematically before testing |
| Integration breaks aircraft builds | Use #ifdef AUV_MODE guards; run full autopilot test suite after each change |
| Parameters overflow in Plane.h | Move to g2 parameter group (like existing additions) |

---

## Notes

- **Specific Gravity**: Use AP_Baro's existing `BARO_SPEC_GRAV` parameter (default 1.0 for fresh, 1.024 for salt water). Configurable via GCS or parameters.txt.
- **Thrust control**: Throttle (0–100%) already maps to motor speed. TECS controls throttle for depth/pitch coordination.
- **Yaw stability**: AUV may need lower yaw gain than aircraft (less roll authority). Tuned during Phase 4 testing.
- **Barometer type configuration**: Set via `barometer.set_type(0, AP_Baro::BARO_TYPE_WATER)` during init, or configure via parameters if available.