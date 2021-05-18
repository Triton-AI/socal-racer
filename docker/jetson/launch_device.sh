#!/bin/bash
docker run --name socalracer -it --rm \
 --runtime=nvidia \
 --privileged \
 --net=host \
haoru233/socal-racer:jetson