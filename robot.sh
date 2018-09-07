#!/usr/bin/env bash
# This script manages deployment of the Magellan stack on the NUC

pushd $(dirname $BASH_SOURCE) > /dev/null

source robot.env
source dockerenv.sh
if [ $? != 0 ]
then
    echo "Couldn't setup dockerenv"
    exit 1
fi

case $1 in
    start)
        shift
        docker run \
            -d \
            -it \
            --privileged \
            --name ${CONTAINER_NAME} \
            --net host \
            -v /dev/bus/usb:/dev/bus/usb \
            -e ROS_IP=${ROBOT_IP} \
            ${IMAGE_NAME}:dev \
            ${@:-$DEFAULT_LAUNCH}

        if [ $? -eq 0 ]
        then
            $0 watch
        else
            echo "Error starting container"
        fi
    ;;

    stop)
        docker stop ${CONTAINER_NAME} &> /dev/null
        docker rm ${CONTAINER_NAME} &> /dev/null
    ;;

    shell)
        docker exec -it ${CONTAINER_NAME} /bin/bash
    ;;

    watch)
        docker logs -f ${CONTAINER_NAME}
    ;;

    deploy)
        $0 stop
        docker build -t ${IMAGE_NAME}:dev .
        $0 start
    ;;

    *)
        echo "Usage: robot.sh [start|stop|deploy|watch]"
    ;;
esac
