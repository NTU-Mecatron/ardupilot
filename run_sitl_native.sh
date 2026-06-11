OUT_PORT=14555
python3 Tools/autotest/sim_vehicle.py -v ArduSub --out udp:127.0.0.1:$OUT_PORT --out udp:${QGC_IP:-127.0.0.1}:14550 -L SGMarinaBarrage --mavproxy-args="--streamrate=-1"