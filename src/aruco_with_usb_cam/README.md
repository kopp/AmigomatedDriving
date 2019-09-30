See
[here](https://www.uco.es/investiga/grupos/ava/node/26)
for more info on Aruco

# aruco_ros

There are two launch files in this package that simplify to work with
`aruco_ros`.  They are written in a way that they work nicely with
`usb_cam_receiver`; i.e. it should be possible to e.g.

    roslaunch usb_cam_receiver edimax_ic_3115w_rectified_standalone.launch
    roslaunch aruco_with_usb_cam track_single_marker.launch

In both cases, a topic `result` publishes the camera image with an overlay of
the found Aruco Marker.


## launch/publish_pose_of_markers.launch

Publishes all found Aruco Markers with pose and id.
A script automatically translates the aruco message into a `MarkerArray`, so
that one can easily display the detected markers in rviz.


## launch/track_single_marker.launch

Tracks one single Aruco Marker (specified by ID) and publishes a `/tf` from the
camera frame to the marker frame.


## coordinate frame differences

*Note* that the tf-frame of the marker seems to be different from the frame
displayed in the `result` topic: In the result image, x and y are in the image
plane (y to the right) and z points out of it.  In the tf frame, x and z are in
the image plane, (z goes to the right) and y points out of it.
This is a
[known (and intended) feature](https://github.com/pal-robotics/aruco_ros/issues/19).



# Markers

You can use
[this site](http://chev.me/arucogen/)
to generate Aruco Markers.
