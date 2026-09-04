## Things to modify about mode_takeoff

### update()

- call calc_throttle, calc_nav_pitch, calc_nav_steering

### navigate()

Role: 
- set target altitude, update waypoint or update maintain heading for nav_controller, set target speed.
- only allow takeoff mode when on the surface. else, reject entering mode.
- check what takeoff state is the plane currently in and decide what to do in each mode.
- when accelerating, set speed and pitch.
- when taking_off, it will set target altitude and update waypoint/heading for nav_controller using initial_heading_cd or loiter radius.