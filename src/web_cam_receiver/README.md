# Package to publish images that were received by a webcam to a ros topic.


# Usage

    % rosrun web_cam_receiver web_cam_receiver_node

This will publish the images from a webcam attached at 10.0.126.9 to
`/image_raw` in the format `sensor_msgs/Image`.  To easily display this, use

    % rosrun image_view image_view image:=/image_raw


# TODO

- Use timeout if cam is not accessible
- Parameterize topic name, IP, username/password, loop rate, frame id


# Compatibility

This was tested with the following web cams:

- Edimax ic 3115w, all resolutions QVGA, VGA, SXVGA


# Calibrate

Follow
[this tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
for calibration.

Note, that when printing
[the provided checkerboard](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=view&target=check-108.pdf)
on a normal printer, it will probably not be printed to scale.  Measure the
length of one grid cell!  In my case (DinA4 printer) I ended up with the
command line call

    % rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0254

Note also, that the output of the calibration is a tarball containing images
and a file `ost.txt`.  In order to convert that to YAML, *rename* it to
`calib.ini` (the `.ini` is the important part here) and then run e.g.

    % rosrun camera_calibration_parsers convert calib.ini edimax_ic_3115w_sxvga.yml

I had to manually change the name from `narrow_stereo` to `edimax_ic_3115w`
