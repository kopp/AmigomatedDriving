Webcam



# basics

We provide the webcam Edimax IC-3115W.
You can connect to them using either ethernet or WiFi.
They will use the same WiFi as the robots do.


# login

user name `admin`, password `1234`


# image

In ros, use the provided package `web_cam_receiver`, which will publish the
image as `sensor_msgs/Image`.

To access the live image in a web browser, go to

    http://<IP adress of camera>/snapshot.cgi

A live image can be fetched using e.g. `wget` via

    wget --user=admin --password=1234 http://<IP addres of camera>/snapshot.jpg

for example

    wget --user=admin --password=1234 http://10.0.126.9/snapshot.jpg


# configure

To access the configuration, go to `http://<IP adress of camera>`.
