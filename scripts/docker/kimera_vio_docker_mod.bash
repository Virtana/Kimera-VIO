#!/bin/bash

# Allow X server connection
xhost +local:root
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/home/shiva/Datasets/V2_01_easy:/data/datasets/Euroc" \
    kimera_local_shiv
# Disallow X server connection
xhost -local:root
