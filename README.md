# Ardupilot fork by Mecatron

This fork contains custom frame configs stored at [AP_Motors6DOF](libraries/AP_Motors/AP_Motors6DOF.cpp) for Mecatron use.

It also contains guide for running ArduSub SITL (Software In The Loop) so that you can run pixhawk package without the need of a physical pixhawk.
However, if you do not intend to run SITL (simulation), ignore all SITL related commands and just follow the firmware build and upload guide.

**Table of Contents**
- [Installation](#installation)
- [Build natively](#build-natively) (RECOMMENDED)
- [Build with Docker](#build-with-docker)
- [Uploading firmware](#uploading-firmware)
- [Uploading parameters](#uploading-parameters)
- [Running SITL](#running-sitl)
- [Available frames](#available-frames)

## Installation

Create a general folder if you have not done so:
```bash
mkdir ~/ardupilot
```

Clone the repository:
```bash
cd ~/ardupilot
git clone --recursive -b Sub-4.5 https://github.com/NTU-Mecatron/ardupilot.git sub-4.5
cd sub-4.5
```

## Build natively (RECOMMENDED)

Instructions are extracted from [Setting up the Build Environment (Linux/Ubuntu)](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux):

```
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Then, configure the build for Software-In-The-Loop:

```bash
./waf configure --board=sitl && ./waf sub
```

## Build with Docker (only encouraged for Jetson use to upload firmware, not any other use cases)

### Setup for the first time build

For first time setup on Jetson, you may need to enable Docker access for your user:

```bash
sudo usermod -aG docker $USER
```

> Note: You may need to restart Jetson for the changes to take effect.

Go to the root of the repo:

```bash
cd ~/ardupilot/sub-4.5
```

Build the Docker image:

```bash
docker build --rm -t ardupilot-dev .
```

Configure the build:

```bash
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf configure --board=Pixhawk6C
```

> Run `docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf list_boards` to see the list of supported boards.

### Subsequent builds

Whenever you make changes to the code, you only need to run the following command to build the firmware (if you already followed the setup above):

```bash
# The above command is only for configuring the build
# You need to run this command to actually build it
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf sub
```

### Uploading firmware

To upload the firmware to the Pixhawk 6C (which is usually at port `/dev/ttyACM0` and `/dev/ttyACM1`), run:

```bash
docker run --rm -it --privileged -v $PWD:/ardupilot ardupilot-dev ./waf --upload-port="/dev/ttyACM0" --upload sub
```

## Uploading parameters

We have created a parameter file [pix6c.parm](params/pix6c.parm) for custom use at Mecatron. This parameter file is working on the assumption that pin 1 for gimbal, pin 2 for marker, pin 3 for torpedo, pin 4 for gripper. Meaning pin 1 and 4 are pwm style, while pin 2 and 3 are relay style.

To upload the parameter file to the Pixhawk 6C, run:

```bash
mavproxy.py
param load <absolute_path_to_pix6c.parm>
reboot
```

> Note: You may need to add `--master=udp:<ip_address>:<port>` if you are using UDP connection, or `--master=/dev/ttyACM0` if you are using serial connection.

If it throws an error "Unable to find parameter RELAY10_PIN", this is because it is a hidden parameter that only appears after you set the RELAY10_FUNCTION to something other than 0. To fix, you need to unplug and replug the Pixhawk (or reboot the Jetson) and try again.

## Running SITL

First, navigate to the ardupilot folder (else, do your own relative paths for the below instructions):
```bash
cd ~/ardupilot
```
Please grant access to all the scripts below by running `chmod +x <script_path>` once. Feel free to edit the scripts to modify the IP address and port if needed.

### Native SITL (No JSON backend)

```bash
./sub-4.5/run_sitl_native.sh
```

### JSON SITL (With JSON backend such as UnityMDS)

You will need to set the environment variable `AP_JSON_IP` to the IP address of the JSON backend server in the `.profile` file. This IP address is where the JSON backend is running (e.g. UnityMDS). If Linux, it should be `127.0.0.1`. If Windows, it should be the Windows WSL2 IP address (usually something like `172.x.x.x`).

Replace `<JSON_BACKEND_IP_ADDRESS>` with the actual IP address of your JSON backend server and run the following command:
```bash
echo 'export AP_JSON_IP=<JSON_BACKEND_IP_ADDRESS>' >> ~/.profile && source ~/.profile
```

Then run the JSON backend and the SITL instance: (order doesn't matter)
```bash
./sub-4.5/run_sitl_json.sh
```

> Note: Remember to enable all firewall rules with Unity if using Windows.

### Some explanations

`--out` flag is used to specify the IP address and port to send the MAVLink messages to. If you are running your pixhawk package in WSL2, you need to run `ifconfig` in WSL2 to find out its IP address and use that IP address.
If you are running the package in Docker, you might need to add `-p 14550` flag when running the container, or add the port manually, and use `127.0.0.1` as the IP address.

## Available frames

For Kevin bot:

```cpp
case SUB_FRAME_CUSTOM:
  //                 Motor #              Roll Factor     Pitch Factor    Yaw Factor      Throttle Factor     Forward Factor      Lateral Factor  Testing Order
  // For Primary bot
  _frame_class_string = "CUSTOM_PRIMARY";
  add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              -1.0f,          0,                  1.0f,               0,              1);
  add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              1.0f,           0,                  1.0f,               0,              2);
  add_motor_raw_6dof(AP_MOTORS_MOT_3,     1.0f,           -1.0f,          -0.5,           -1.0f,              0,                  -1.0f,          3);
  add_motor_raw_6dof(AP_MOTORS_MOT_4,     -1.0f,          -1.0f,          0.5,            -1.0f,              0,                  1.0f,           4);
  add_motor_raw_6dof(AP_MOTORS_MOT_5,     1.0f,           1.0f,           -0.5,           -1.0f,              0,                  1.0f,           5);
  add_motor_raw_6dof(AP_MOTORS_MOT_6,     -1.0f,          1.0f,           0.5,            -1.0f,              0,                  -1.0f,          6);
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