JSON_BACKEND_IP_ADDRESS=172.24.160.1

# Parse command-line arguments
INSTANCE=0
while [[ $# -gt 0 ]]; do
    case $1 in
        -I)
            INSTANCE="$2"
            shift 2
            ;;
        *)
            shift
            ;;
    esac
done


docker run --rm -it \
    --network host \
    -e AP_JSON_IP=$JSON_BACKEND_IP_ADDRESS \
    -v $PWD:/ardupilot \
    ardupilot-dev \
    ./run_sitl_json.sh -I $INSTANCE