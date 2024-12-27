# Ardupilot fork by Mecatron

This fork contains custom frame configs stored at [AP_Motors6DOF](libraries/AP_Motors/AP_Motors6DOF.cpp) for Mecatron use.

## Installation

```bash
cd ~/
git clone --recursive -b Sub-4.5 https://github.com/NTU-Mecatron/ardupilot.git
cd ardupilot
```

## Build

For first time setup on Jetson, you may need to enable Docker access for your user:

```bash
sudo usermod -aG docker $USER
```

> Note: You may need to restart your computer for the changes to take effect.

Then, build the Docker image (should only be done once):

```bash
docker build --rm -t ardupilot-dev .
```

Build ardusub image for Pixhawk 6C:

```bash
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf configure --board=Pixhawk6C
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf sub
```

To upload the firmware to the Pixhawk 6C (which is usually at port `/dev/ttyACM0` and `/dev/ttyACM1`), run:

```bash
docker run --rm -it --privileged -v $PWD:/ardupilot ardupilot-dev ./waf --upload-port="/dev/ttyACM0" --upload sub
```