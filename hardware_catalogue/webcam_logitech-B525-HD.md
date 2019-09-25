# Webcam Logitech B525 HD

![Logitech B525 HD](webcam_logitech-b525-hd.jpg)


# setup

Connect via usb :-)


# use within ros

Run

    rosrun usb_cam usb_cam_node

to grab the images from USB and forward to ROS.
By default, it will publish under `/usb_cam`, i.e. `/usb_cam/image_raw`.

To test this, run

    rosrun image_view image_view image:=/usb_cam/image_raw


# configuration

By default, this will display the image from `/dev/video0`; to change this, use

    rosparam set /usb_cam/video_device /dev/bar
