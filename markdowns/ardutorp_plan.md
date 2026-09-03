# ArduTorp Implementation Plan

ArduTorp is built on the foundation of ArduPlane, but up to 80% of the code needs to be revamped to make it work for torpedo-shaped AUVs.

Original summary of the ArduPlane is written [here](original_code_structure.md). This document outlines the master plan and all the existing functions that need to be modified for the torpedo use case.

Feel free to delete unnecessary code which is unique to plane (e.g., landing gear, flaps, etc.) and add new code for AUV-specific features (e.g., buoyancy compensation, fin allocation, etc.). This branch is designed only for torpedo-shaped AUVs.

---

## Control Flow Impact Analysis

**AUV Modifications Fit Into**:
1. Replace TECS with a AC_PID controller for altitude control
2. **update_control_mode()** (Fast) → calls **mode->update()** — Add yaw calculation instead of roll
3. **stabilize()** (Fast) — Route yaw instead of roll, keep pitch unchanged
4. **servos_output()** (Fast) — Replace mixing with fin allocation, read pitch/yaw/roll outputs

---

## Implementation strategy 

### Replace TECS with a new AP_AltitudeControl class

This class should utilize the [AC_PID](../libraries/AC_PID/AC_PID.h) class to compute desired pitch given altitude setpoint using a simple P controller. To be located under [APM_Control](../libraries/APM_Control/) folder. Minimum required functions:
- get and set altitude setpoint
- get target pitch (to be used by plane's pitch controller; no argument)

I want one additional parameter for this class: `BUOYANCY_FF` which is a disturbance feedforward term to account for the vehicle's buoyancy. This will help the controller maintain depth.

Proceed to replace TECS completely with this new class. Speed control will be handled by a speed-throttle PID.

### Integrate AR_AttitudeControl with L1_Controller

[AR_AttitudeControl](../libraries/APM_Control/AR_AttitudeControl.h) is a class used to compute steering and throttle using AC_PID, and is already currently used in `Rover`. 

It requires desired lat_acc (lateral acceleration) and speed as inputs. Lat_acc is usually only returned from the L1 controller which runs at 10hz when in auto mode or similar modes (in control_mode->navigate()). Speed is typically set by the pilot or mission planner in control_mode->update().

`AR_AttitudeControl` will be used to augment the existing `L1_controller` aka nav_controller. Strategy: call nav_controller->lateral_acceleration() and pass it to AR_AttitudeControl to compute steering.

Remove existing yaw and steering control.

### Create a new servo mixer for fins allocation

After desired roll, pitch and steering are computed, they will be sent to a new fin allocation mixer which will compute the desired deflection for each fin. This will replace the existing mixing logic in `servos_output()`.

There should be so many new parameters to define each fin's effectiveness in pitch, roll and yaw. The mixer will then compute the desired deflection for each fin based on the desired roll, pitch and yaw.

> Note: This step is to be done last. Now, we assume that the AUV follows a T-fin configuration (like an aircraft) and we can use the existing mixing logic to test out the control flow.

### Modify the current codebase and mode_takeoff as a proof of technology

Utilize the above altitude and attitude controller and modify the existing `mode_takeoff` to test out the control flow. The vehicle should be able to take off (dive) and maintain depth without any waypoint navigation.

### Moving away from T-fin assumption to Arbitrary Fin Allocation System

Fins can be in various configurations (+, X, H, etc.). Need flexible allocation matrix.

*Control Flow Point*: `set_servos()` (Fast ~400Hz) → `servos_output()` — This is where control surface mixing happens.

Example implementation (not verified):

#### Create Fin Allocation System
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

#### Implement Simple Linear Allocation
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

#### Integrate with Servo Output Pipeline
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

#### Add Fin Configuration Parameters
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

#### Define Fin Servo Channel Types
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