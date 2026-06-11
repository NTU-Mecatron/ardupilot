QGC_IP=172.24.160.1

docker run --rm -it \
    --network host \
    -v $PWD:/ardupilot \
    -e QGC_IP=$QGC_IP \
    ardupilot-dev \
    ./run_sitl_native.sh