JSON_BACKEND_IP_ADDRESS=172.24.160.1

docker run --rm -it \
    --network host \
    -e AP_JSON_IP=$JSON_BACKEND_IP_ADDRESS \
    -v $PWD:/ardupilot \
    ardupilot-dev \
    ./run_sitl_json.sh