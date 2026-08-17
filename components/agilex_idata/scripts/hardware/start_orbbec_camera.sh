#!/usr/bin/env bash
FRONT_SERIAL="${FRONT_SERIAL:-CC136530352}"
LEFT_SERIAL="${LEFT_SERIAL:-CC1365303LY}"
RIGHT_SERIAL="${RIGHT_SERIAL:-CC136530160}"

exec ros2 launch orbbec_camera multi_camera.launch.py \
  front_serial:="${FRONT_SERIAL}" \
  left_serial:="${LEFT_SERIAL}" \
  right_serial:="${RIGHT_SERIAL}"
