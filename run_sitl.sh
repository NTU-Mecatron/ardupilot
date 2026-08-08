#!/bin/bash

# Usage: ./run_sitl.sh [--json|--native] [--docker] [-I INSTANCE]

JSON_BACKEND_SIM_IP=127.0.0.1
QGC_IP=127.0.0.1
QGC_PORT=14550
MAVROS_IP=127.0.0.1
MAVROS_PORT=14555   # Default, but will be automatically incremented by 10 for each instance (see below)

# Default values
MODE="json"
USE_DOCKER=true
INSTANCE=0

# Parse command-line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --json)
            MODE="json"
            shift
            ;;
        --native)
            MODE="native"
            shift
            ;;
        --no-docker)
            USE_DOCKER=false
            shift
            ;;
        -I)
            INSTANCE="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [--json|--native] [--no-docker] [-I INSTANCE]"
            echo ""
            echo "Options:"
            echo "  --json       Use external physics engine via json protocol (default)"
            echo "  --native     Use native, built-in sitl engine"
            echo "  --no-docker  Run outside Docker container (default: inside Docker)"
            echo "  -I INSTANCE  Instance number (default: 0)"
            echo "  -h, --help   Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Calculate port and sysid based on instance
MAVROS_PORT=$((14555 + INSTANCE * 10))
SYSID=$((1 + INSTANCE))

# Check if sim_vehicle.py exists
if [ ! -f "Tools/autotest/sim_vehicle.py" ]; then
    echo "Warning: Tools/autotest/sim_vehicle.py not found!"
    echo "Make sure you are running from the root directory containing this script."
    echo "This is so that you can reuse the eeprom file containing parameters saved from last time."
    exit 1
fi

# Build the base python command
PYTHON_CMD="python3 Tools/autotest/sim_vehicle.py \
    -v ArduCopter -f X \
    --out udp:$MAVROS_IP:$MAVROS_PORT \
    --out udp:$QGC_IP:$QGC_PORT \
    -L SGMarinaBarrage \
    -I $INSTANCE \
    --sysid $SYSID \
    --mavproxy-args=\"--streamrate=-1\""

# Add mode-specific options
if [ "$MODE" = "json" ]; then
    PYTHON_CMD="$PYTHON_CMD \
    --model JSON:$JSON_BACKEND_SIM_IP \
    --add-param-file=params/sitl_json.parm"
fi

# Execute the command
if [ "$USE_DOCKER" = true ]; then
    docker run --rm -it \
        --network host \
        -v $PWD:/ardupilot \
        ardupilot-dev \
        bash -c "$PYTHON_CMD"
else
    eval "$PYTHON_CMD"
fi
