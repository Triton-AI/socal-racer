#!/bin/bash

XSOCK=/tmp/.X11-unix
docker run --name socalracer -it --rm \
 --runtime=nvidia \
 -e DISPLAY=$DISPLAY \
 -v $XSOCK:$XSOCK \
 -v $HOME/.Xauthority:/root/.Xauthority \
 --privileged \
 --net=host \
haoru233/socal-racer:base