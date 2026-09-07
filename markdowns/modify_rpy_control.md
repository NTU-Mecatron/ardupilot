## Modify RPY Control to make it work for Torp AUV

Original RPY controls implemented under [APM_Control](../libraries/APM_Control/) are for airplanes.

Task: make modifications to make it work for underwater torpedo-shaped AUVs.
- Roll, pitch, yaw all remove reference to airspeed, bank angle, etc..
- Each axis is assumed to be independent from each other.
- Yaw to turn only involves rudder (the servo output), do not depend on roll.