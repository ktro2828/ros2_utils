#!/bin/bash

CAM_NUM:=$1

CAM_TOPIC="/sensing/camera/camera${CAM_NUM}/image_rect_color"

ros2 run image_transport republish compressoed --ros-args --remap /in/compressed:=${CAM_TOPIC}/compressed  --remap out:=${CAM_TOPIC}
