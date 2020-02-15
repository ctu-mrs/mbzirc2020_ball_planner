#!/bin/bash

path="/home/\$(optenv USER mrs)/bag_files/latest/"

exclude=(
# raw oustr data
'(.*)ouster_driver(.*)'
# bluefox
'/$(arg UAV_NAME)/bluefox/image_raw'
'/$(arg UAV_NAME)/bluefox/image_raw/compressed/(.*)'
'/$(arg UAV_NAME)/bluefox/image_raw/compressedDepth(.*)'
# bluefox_optflow
'/$(arg UAV_NAME)/bluefox_optflow/image_raw'
'/$(arg UAV_NAME)/bluefox_optflow/image_raw/compressed/(.*)'
'/$(arg UAV_NAME)/bluefox_optflow/image_raw/compressedDepth(.*)'
# bluefox3_front
'/$(arg UAV_NAME)/bluefox3_front/image_raw'
'/$(arg UAV_NAME)/bluefox3_front/image_raw/compressed/(.*)'
'/$(arg UAV_NAME)/bluefox3_front/image_raw/compressedDepth(.*)'
# Realsense t265
'/$(arg UAV_NAME)/rs_t265/fisheye(.*)'
# Realsesne d435
'(.*)rs_d435(.*)depth_to_infra(.*)'
'(.*)rs_d435(.*)depth_to_color/image_raw'
'(.*)rs_d435(.*)depth_to_color(.*)compressed'
'(.*)rs_d435(.*)depth_to_color(.*)compressed/(.*)'
'(.*)rs_d435(.*)depth_to_color(.*)compressedDepth/(.*)'
'(.*)rs_d435(.*)color/image_raw'
'(.*)rs_d435(.*)color/image_rect_color'
'(.*)rs_d435(.*)color(.*)compressed/(.*)'
'(.*)rs_d435(.*)color(.*)compressedDepth(.*)'
'(.*)rs_d435(.*)depth/image_raw'
'(.*)rs_d435(.*)depth/image_rect_raw'
'(.*)rs_d435(.*)depth/image_rect_raw/compressed'
'(.*)rs_d435(.*)depth/image_rect_raw/compressed/(.*)'
'(.*)rs_d435(.*)depth/image_rect_raw/compressedDepth/(.*)'
# '(.*)rs_d435(.*)infra(.*)/image_raw'
# '(.*)rs_d435(.*)infra(.*)/image_rect_raw'
# '(.*)rs_d435(.*)infra(.*)compressed/(.*)'
'(.*)rs_d435(.*)infra(.*)'
# Depth detect compressed
'(.*)depth_detect(.*)processed_depthmap'
'(.*)depth_detect(.*)processed_depthmap/compressed(.*)'
'(.*)depth_detect(.*)processed_depthmap/compressedDepth/(.*)'
# Object detect compressed
'(.*)object_detect(.*)debug_image'
'(.*)object_detect(.*)debug_image/compressed/(.*)'
'(.*)object_detect(.*)debug_image/compressedDepth(.*)'
# Every theora message
'(.*)theora(.*)'
# Every h264 message
'(.*)h264(.*)'
)

# file's header
filename=`mktemp`
echo "<launch>" > "$filename"
echo "<arg name=\"UAV_NAME\" default=\"\$(env UAV_NAME)\" />" >> "$filename"
echo "<group ns=\"\$(arg UAV_NAME)\">" >> "$filename"

echo -n "<node pkg=\"rosbag\" type=\"record\" name=\"rosbag_record\" args=\"-o $path -a" >> "$filename"

# if there is anything to exclude
if [ "${#exclude[*]}" -gt 0 ]; then

  echo -n " -x " >> "$filename"

  # list all the string and separate the with |
  for ((i=0; i < ${#exclude[*]}; i++));
  do
    echo -n "${exclude[$i]}" >> "$filename"
    if [ "$i" -lt "$( expr ${#exclude[*]} - 1)" ]; then
      echo -n "|" >> "$filename"
    fi
  done

fi

echo "\" />" >> "$filename"

# file's footer
echo "</group>" >> "$filename"
echo "</launch>" >> "$filename"

cat $filename
roslaunch $filename
