Perform a simple SLAM algorithm with g2o on a bag file which contains odometry
and located aruco markers.


# How to use

- Read this file completely before you start to experiment.
- Print aruco markers of the same size but with different ids and attach them
  to random places in your environment.
- Use your robot to drive around this environment and capture all markers with
  its camera.
- Record the output from the robot as bag file -- most notably (topics assuming
  you used the standard setup from `amigobot`)
  - robot poses (`/RosAria/pose`)
  - aruco marker poses (`/web_cam/aruco_marker_publisher/markers`)
  - tf (`/tf /tf_static`)
- Run the slam tool from this package on the bag file.  This execute a simple
  slam; the output is the optimized markers position in a new map.


# Add aruco markers to bag file

If you have a bagfile that does not contain the extracted aruco markers, you
can re-simulate them offline easily.  Run the following in separate terminals:

    roscore
    rosparam load param_dump_of_your_recording.param && rosparam set use_sim_time true
    rosbag play --clock --pause --rate 1 --topics /RosAria/pose /tf /tf_static /web_cam/camera_info /web_cam/image_rect_color -- bag_file_without_aruco_markers.bag
    rosbag record -O bag_file_with_markers.bag /RosAria/pose /tf /tf_static /web_cam/aruco_marker_publisher/markers
    roslaunch aruco_with_web_cam publish_pose_of_markers.launch

Then hit space in the first terminal to start playback.  Once the playback is
done, hit Control+C in the second to stop recording.

*Note*: Depending on the power of your computer, it may be a good idea to set
the `--rate` value to something lower (e.g. 0.2).
