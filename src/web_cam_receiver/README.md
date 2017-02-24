Package to publish images that were received by a webcam to a ros topic.


# Usage

Execute

    rosrun web_cam_receiver web_cam_receiver_node

This will publish the images from a webcam attached at 10.0.126.9 to
`/web_cam_image` in the format `sensor_msgs/Image`.


# TODO

- Use timeout if cam is not accessible
- Parameterize topic name, IP, username/password, loop rate, frame id


# Compatibility

This was tested with the following web cams:

- Edimax ic 3115w, all resolutions QVGA, VGA, SXVGA
