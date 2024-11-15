#!/bin/bash
xhost local:docker
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker network inspect ros-net >/dev/null 2>&1 || \
    docker network create ros-net --subnet=172.18.0.0/16

docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --gpus all \
    --ip 172.18.0.2 \
    -p 11311:11311 \
    --network ros-net \
    dragonfly-sim:latest --drones 4 --location BALLOON_FIESTA
