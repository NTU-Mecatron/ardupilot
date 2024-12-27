# Ardupilot fork by Mecatron

This fork contains custom frame configs stored at [AP_Motors6DOF](libraries/AP_Motors/AP_Motors6DOF.cpp) for Mecatron use.

## Available frames

For Kevin bot:

```cpp
case SUB_FRAME_CUSTOM:
  //                 Motor #              Roll Factor     Pitch Factor    Yaw Factor      Throttle Factor     Forward Factor      Lateral Factor  Testing Order
  // For Primary bot
  _frame_class_string = "CUSTOM_PRIMARY";
  add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              -1.0f,          0,                  1.0f,               0,              1);
  add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              1.0f,           0,                  1.0f,               0,              2);
  add_motor_raw_6dof(AP_MOTORS_MOT_3,     1.0f,           -1.0f,          0,              -1.0f,              0,                  -1.0f,          3);
  add_motor_raw_6dof(AP_MOTORS_MOT_4,     -1.0f,          -1.0f,          0,              -1.0f,              0,                  1.0f,           4);
  add_motor_raw_6dof(AP_MOTORS_MOT_5,     1.0f,           1.0f,           0,              -1.0f,              0,                  1.0f,           5);
  add_motor_raw_6dof(AP_MOTORS_MOT_6,     -1.0f,          1.0f,           0,              -1.0f,              0,                  -1.0f,          6);
  break;
```

For Lucy frame:

```cpp
case SUB_FRAME_SIMPLEROV_5:
  //                 Motor #              Roll Factor     Pitch Factor    Yaw Factor      Throttle Factor     Forward Factor      Lateral Factor  Testing Order
  // For Secondary bot
  _frame_class_string = "CUSTOM_SECONDARY";
  add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,               -1.0f,          0,                  1.0f,               0,              1);
  add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,               1.0f,           0,                  1.0f,               0,              2);
  add_motor_raw_6dof(AP_MOTORS_MOT_3,     1.0f,           -1.0f,           0,              -1.0f,              0,                  0,              3);
  add_motor_raw_6dof(AP_MOTORS_MOT_4,     -1.0f,          -1.0f,           0,              -1.0f,              0,                  0,              4);
  add_motor_raw_6dof(AP_MOTORS_MOT_5,     0,              1.0f,            0,              -1.0f,              0,                  0,              5);
  break; 
```

## Installation

```bash
cd ~/
git clone --recursive -b Sub-4.5 https://github.com/NTU-Mecatron/ardupilot.git
cd ardupilot
```

## Setup for the first time build

For first time setup on Jetson, you may need to enable Docker access for your user:

```bash
sudo usermod -aG docker $USER
```

> Note: You may need to restart Jetson for the changes to take effect.

Then, build the Docker image:

```bash
docker build --rm -t ardupilot-dev .
```

Configure the build for Pixhawk 6C: (You can change the board to other supported boards, such as Pixhawk4 or sitl)

```bash
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf configure --board=Pixhawk6C
```

## Subsequent builds

Whenever you make changes to the code, you only need to run the following command to build the firmware (if you already followed the setup above):

```bash
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf sub
```

To upload the firmware to the Pixhawk 6C (which is usually at port `/dev/ttyACM0` and `/dev/ttyACM1`), run:

```bash
docker run --rm -it --privileged -v $PWD:/ardupilot ardupilot-dev ./waf --upload-port="/dev/ttyACM0" --upload sub
```