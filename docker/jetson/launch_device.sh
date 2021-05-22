#!/bin/bash
docker run --name socalracer -it --rm \
 --mount type=volume,source=socalracer-devel,dst=/root \
 --runtime=nvidia \
 --privileged \
 --net=host \
haoru233/socal-racer:jetson