#!/bin/bash

cd `dirname "$0"`

ROOTDIR=`pwd`
export WEBSOCKET_URL="wss://car_name_it_your_way.ngrok.lcas.group:4433/carws"


_shutdown_server() {
	echo "shutting down $nginx_pid" >&2
	sleep 2	
	kill $nginx_pid  > /dev/null 2>&1
}

trap _shutdown_server 1 2 3 6 15

nginx -p "${ROOTDIR}" -c "${ROOTDIR}/conf/nginx.conf" &

nginx_pid=$!

python "${ROOTDIR}/callarobot.py" 





