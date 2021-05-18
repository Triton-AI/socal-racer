#!/bin/bash
docker run --name socal-racer --rm -it \
    --runtime=nvidia \
    --net=host \
    haoru233/socal-racer:jetson