#!/bin/bash
docker build -t rslidar_sdk -f "$(dirname "$0")/Dockerfile" "$(dirname "$0")/.."

docker run --rm \
    --net=host \
    --ipc=host \
    -e DISPLAY=:0 \
    -e XAUTHORITY=/root/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$HOME/.Xauthority:/root/.Xauthority:ro" \
    rslidar_sdk
