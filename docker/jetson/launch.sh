#!/bin/bash
docker run --name socal-racer --rm -it \
    --mount type=volume,source=socalracer-devel,dst=/root \
    --runtime=nvidia \
    --net=host \
    haoru233/socal-racer:jetson