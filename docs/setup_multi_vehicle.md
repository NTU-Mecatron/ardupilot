# Multi-vehicle SITL setup

One script clones all three vehicles into `~/ardupilot` and builds SITL for each of them with Docker:

| Directory | Branch | Vehicle |
| --- | --- | --- |
| `~/ardupilot/uuv-sub-4.5` | `Sub-4.5` | UUV |
| `~/ardupilot/usv-sub-4.5` | `Sub-4.5` | USV |
| `~/ardupilot/copter-4.5` | `Copter-4.5` | UAV |

## Prerequisites

Install Docker Engine natively (do **not** use Docker Desktop), then give your user access to it:

```bash
sudo usermod -aG docker $USER
```

> Note: You may need to restart your laptop for the changes to take effect.

## Running the script

```bash
mkdir -p ~/ardupilot
cd ~/ardupilot
wget https://github.com/NTU-Mecatron/miscellaneous/releases/download/ardupilot/setup.sh
chmod +x setup.sh
./setup.sh
```

## After it finishes

Run SITL from the root of whichever vehicle you want:

```bash
cd ~/ardupilot/uuv-sub-4.5
./run_sitl.sh
```

See [Running SITL](../README.md#running-sitl) for the options, and rebuild after code changes with:

```bash
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf sub      # or ./waf copter
```
