# ArduPlane Control Flow: From Commands to Servo Outputs

## Overview
This document explains how ArduPlane processes inputs (radio commands or waypoint commands) and converts them into servo and throttle outputs. The process occurs through a scheduled task system running at various frequencies.

## Main Scheduler (ArduPlane.cpp)

The scheduler is the heart of the system. It runs tasks at different frequencies based on priority:

### Fast Tasks (Run every loop)
1. **ahrs_update()** - Updates attitude/heading reference system (AHRS)
2. **update_control_mode()** - Mode-specific control updates
3. **stabilize()** - Runs attitude stabilization controllers
4. **set_servos()** - Outputs to servos and motors

### Medium Rate Tasks (10-50 Hz)
- **read_radio()** (50 Hz) - Reads RC input from transmitter
- **update_speed_height()** (50 Hz) - TECS speed/height controller
- **navigate()** (10 Hz) - Waypoint navigation and path following
- **calc_airspeed_errors()** (10 Hz) - Calculates airspeed errors for TECS
- **update_alt()** (10 Hz) - Altitude tracking and updates

## Control Flow Pipeline

### 1. Input Stage

#### Radio Input (radio.cpp)
```
read_radio() @ 50 Hz
├── Reads RC channels (roll, pitch, yaw, throttle)
├── set_control_channels() - Maps RC inputs to control channels
├── Applies dead zones
└── rudder_arm_disarm_check() - Handles arming via rudder stick
```

#### Waypoint Input (navigation.cpp, commands_logic.cpp)
```
navigate() @ 10 Hz
├── mission.update() - Updates current mission command
├── Calculates waypoint distance and bearing
├── Updates loiter angle tracking
└── Calls control_mode->navigate() for mode-specific navigation
```

### 2. AHRS Update (ArduPlane.cpp)
```
ahrs_update() @ Fast (>100 Hz)
├── ahrs.update() - Fuses IMU, GPS, compass data
├── Calculates roll/pitch limits based on airspeed
├── Updates gyro integration for steering
└── Logs IMU data
```

### 3. Mode-Specific Control (control_modes.cpp, mode_*.cpp)

Each flight mode implements its own control logic:

#### AUTO Mode Example (mode_auto.cpp)
```
ModeAuto::update()
├── Checks mission state
├── For NAV_TAKEOFF:
│   ├── takeoff_calc_roll() - Calculates desired roll
│   ├── takeoff_calc_pitch() - Calculates desired pitch
│   └── calc_throttle() - Calculates throttle from TECS
├── For NAV_LAND:
│   ├── calc_nav_roll() - Calculates navigation roll
│   ├── calc_nav_pitch() - Calculates navigation pitch
│   └── calc_throttle() or suppress throttle if landing complete
└── For normal waypoints:
    ├── calc_nav_roll() - L1 controller for roll
    ├── calc_nav_pitch() - Calculate pitch for altitude tracking
    └── calc_throttle() - TECS throttle output
```

#### Manual/FBWA Modes
```
- Read RC inputs directly
- Apply stabilization to stick inputs
- Manual throttle passthrough or FBWA throttle mixing
```

### 4. Speed/Height Controller (ArduPlane.cpp)
```
update_speed_height() @ 50 Hz
└── TECS_controller.update_50hz()
    ├── Calculates energy error (kinetic + potential)
    ├── Determines throttle to maintain target airspeed
    └── Determines pitch to maintain target altitude
```

### 5. Attitude Stabilization (Attitude.cpp)

The `stabilize()` function is the core of attitude control:

```
stabilize() @ Fast
├── For Manual/Training modes:
│   └── Direct RC passthrough (minimal processing)
│
├── For all other modes:
│   ├── stabilize_roll()
│   │   ├── calc_speed_scaler() - Adjusts gains based on airspeed
│   │   ├── rollController.get_servo_out(nav_roll_cd - current_roll)
│   │   │   ├── P term: proportional to roll error
│   │   │   ├── I term: integrated roll error
│   │   │   ├── D term: roll rate damping
│   │   │   └── FF term: feedforward from desired roll rate
│   │   └── SRV_Channels::set_output_scaled(k_aileron, roll_out)
│   │
│   ├── stabilize_pitch()
│   │   ├── Checks for takeoff tail hold
│   │   ├── Adds pitch trim and throttle->pitch compensation
│   │   ├── pitchController.get_servo_out(nav_pitch_cd - current_pitch)
│   │   └── SRV_Channels::set_output_scaled(k_elevator, pitch_out)
│   │
│   └── stabilize_yaw()
│       ├── Determines if ground steering is active
│       ├── calc_nav_yaw_coordinated() - Coordinated turn rudder
│       ├── Applies rudder mixing from roll
│       └── SRV_Channels::set_output_scaled(k_rudder, rudder_out)
│
└── Zeros integrators if on ground with zero throttle
```

**PID Controllers:**
- `rollController` - Maintains desired roll angle
- `pitchController` - Maintains desired pitch angle  
- `yawController` - Provides coordinated turn and directional control
- `TECS_controller` - Total Energy Control System for speed/altitude

### 6. Throttle Calculation (Attitude.cpp, servos.cpp)

```
calc_throttle()
└── commanded_throttle = TECS_controller.get_throttle_demand()

set_throttle() in set_servos()
├── Checks arming state
├── Checks for throttle suppression (on ground)
├── Mode-specific throttle:
│   ├── Manual modes: get_throttle_input() from RC
│   ├── Auto modes: Use TECS throttle
│   └── VTOL modes: quadplane.forward_throttle_pct()
├── Apply battery voltage compensation
├── Apply throttle limits (min/max)
└── SRV_Channels::set_output_scaled(k_throttle, throttle)
```

### 7. Servo Output Stage (servos.cpp)

The `set_servos()` function is called every loop:

```
set_servos() @ Fast
├── SRV_Channels::cork() - Batch outputs
│
├── quadplane.update() - VTOL transitions if enabled
├── landing.override_servos() - Landing overrides if in LAND stage
│
├── set_throttle()
│   ├── Suppress throttle if on ground
│   ├── Apply throttle from mode or RC
│   ├── Apply battery compensation
│   └── Apply min/max limits
│
├── set_servos_flaps()
│   ├── Calculate auto flap based on airspeed/flight stage
│   ├── Read manual flap input from RC
│   ├── Apply flap slew rate limiting
│   └── Output to flaperon mixer
│
├── airbrake_update() - Set airbrakes from reverse thrust or RC
├── throttle_slew_limit() - Rate limit throttle changes
│
└── servos_output()
    ├── servos_twin_engine_mix() - Mix for differential thrust
    ├── channel_function_mixer() - Elevon/V-tail mixing
    ├── dspoiler_update() - Differential spoilers
    ├── SRV_Channels::calc_pwm() - Convert scaled outputs to PWM
    ├── SRV_Channels::output_ch_all() - Write to all channels
    └── SRV_Channels::push() - Send to hardware
```

## Key Data Structures

### Control Variables
- `nav_roll_cd` - Desired roll angle (centidegrees) from navigation
- `nav_pitch_cd` - Desired pitch angle (centidegrees) from navigation
- `roll_limit_cd` - Maximum roll angle limit
- `pitch_limit_min` - Minimum pitch angle limit
- `target_airspeed_cm` - Desired airspeed (cm/s)

### State Variables
- `control_mode` - Current flight mode pointer
- `auto_state` - AUTO mode state (waypoint info, flight stage)
- `flight_stage` - Current phase (NORMAL, TAKEOFF, LAND, etc.)
- `is_flying()` - Whether aircraft is airborne

## Signal Path Examples

### Example 1: Radio Command in FBWA Mode
```
RC Transmitter stick movement
    ↓
read_radio() - Read stick positions
    ↓
ModeTraining::update() or ModeFBWA::update()
    ↓
calc_nav_roll() - Convert stick to desired roll angle
calc_nav_pitch() - Convert stick to desired pitch angle
    ↓
stabilize()
    ├── stabilize_roll() → rollController → aileron PWM
    ├── stabilize_pitch() → pitchController → elevator PWM
    └── stabilize_yaw() → yawController → rudder PWM
    ↓
set_servos()
    ├── set_throttle() - RC throttle passthrough
    └── servos_output() - Apply mixing and output PWM
    ↓
Servo actuators move control surfaces
```

### Example 2: Waypoint Command in AUTO Mode
```
Mission waypoint (lat/lon/alt)
    ↓
navigate() - Calculate bearing and distance to waypoint
    ↓
mission.update() - Update current command
    ↓
ModeAuto::update()
    ↓
calc_nav_roll() - L1 controller calculates roll for path following
calc_nav_pitch() - Calculate pitch for altitude tracking
update_speed_height() - TECS calculates pitch/throttle for altitude/speed
    ↓
stabilize()
    ├── stabilize_roll() → rollController → aileron PWM
    ├── stabilize_pitch() → pitchController → elevator PWM  
    └── stabilize_yaw() → yawController → rudder PWM
    ↓
calc_throttle() - TECS throttle demand
    ↓
set_servos()
    ├── set_throttle() - Apply TECS throttle
    └── servos_output() - Apply mixing and output PWM
    ↓
Servo actuators and ESC/motor respond
```

## Control Loop Frequencies

| Task | Frequency | Purpose |
|------|-----------|---------|
| ahrs_update | Fast (~400Hz) | Sensor fusion |
| stabilize | Fast (~400Hz) | Attitude control |
| set_servos | Fast (~400Hz) | Servo outputs |
| update_control_mode | Fast (~400Hz) | Mode logic |
| read_radio | 50 Hz | RC input |
| update_speed_height | 50 Hz | TECS update |
| navigate | 10 Hz | Waypoint navigation |
| update_GPS | 10 Hz | GPS processing |
| update_compass | 10 Hz | Compass reading |

## Important Controllers

### L1 Controller (AP_L1_Control)
- Calculates lateral navigation commands
- Outputs `nav_roll_cd` for path following
- Handles straight lines and loiters

### TECS Controller (AP_TECS)  
- Total Energy Control System
- Coordinates pitch and throttle for altitude and airspeed control
- Outputs throttle demand and pitch contribution

### PID Controllers (APM_Control)
- **Roll Controller**: Tracks desired roll angle with rate damping
- **Pitch Controller**: Tracks desired pitch angle with rate damping
- **Yaw Controller**: Provides coordinated turns and directional control
- All use speed scaling to adjust gains based on airspeed

## Special Features

### Stick Mixing
- Allows pilot override in auto modes
- `stick_mixing_enabled()` checks if enabled
- `stabilize_stick_mixing_fbw()` adds stick input to nav commands

### Throttle Suppression
- `suppress_throttle()` zeros throttle when on ground
- Prevents propeller activation before takeoff
- Based on altitude, speed, and flight stage

### Flare Control
- During landing flare, special pitch and throttle control
- `landing.is_flaring()` triggers flare mode
- Pitch controlled by `LAND_PITCH_DEG`
- Throttle suppressed or set to minimum

### Differential Thrust
- For twin-engine aircraft
- `servos_twin_engine_mix()` mixes rudder into left/right throttle
- Provides yaw control through differential thrust

## Summary

The control flow in ArduPlane is:

1. **Inputs** → RC or waypoints provide desired state
2. **Navigation** → Convert waypoints to roll/pitch/throttle targets  
3. **AHRS** → Estimate current attitude and position
4. **Mode Logic** → Mode-specific calculation of desired attitudes
5. **TECS** → Energy-based speed and altitude control
6. **Stabilization** → PID controllers convert attitude errors to control surface deflections
7. **Mixing** → Combine control surfaces (elevons, v-tail, differential thrust)
8. **Output** → PWM signals to servos and ESCs

This architecture provides a clean separation between high-level navigation/guidance and low-level attitude control, with the TECS providing the bridge for throttle and altitude coordination.
