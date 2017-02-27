#!/usr/bin/env python
# license removed for brevity
import rospy
import base64
import time
import urllib2

import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



"""
Examples of objects for image frame aquisition from both IP and
physically connected cameras
Requires:
 - opencv (cv2 bindings)
 - numpy
"""


class ipCamera(object):

    def __init__(self, url, user=None, password=None):
        self.url = url
        auth_encoded = base64.encodestring('%s:%s' % (user, password))[:-1]

        self.req = urllib2.Request(self.url)
        self.req.add_header('Authorization', 'Basic %s' % auth_encoded)

    def get_frame(self):
        response = urllib2.urlopen(self.req)
        img_array = np.asarray(bytearray(response.read()), dtype=np.uint8)
        frame = cv2.imdecode(img_array, 1)
        return frame


class Camera(object):

    def __init__(self, camera=0):
        self.cam = cv2.VideoCapture(camera)
        if not self.cam:
            raise Exception("Camera not accessible")

        self.shape = self.get_frame().shape

    def get_frame(self):
        _, frame = self.cam.read()



def WebcamReceiver():
    image_pub = rospy.Publisher("/RosAria/Webcam",Image, queue_size = 1)
    bridge = CvBridge()
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    ip_cam = ipCamera('http://10.0.126.8/snapshot.jpg',user = "admin", password = "1234")

    while not rospy.is_shutdown():
        #        hello_str = "hello world %s" % rospy.get_time()
        #        rospy.loginfo(hello_str)
        #        pub.publish(hello_str)
        frame = ip_cam.get_frame()
        ros_frame = bridge.cv2_to_imgmsg(frame, "bgr8")
        # Display the resulting frame
        #cv2.imshow('frame',frame)
        #cv2.waitKey(30)
        try:
              image_pub.publish(ros_frame)
        except CvBridgeError as e:
              print(e)

        rate.sleep()

if __name__ == '__main__':
    try:
        WebcamReceiver()
    except rospy.ROSInterruptException:
        pass
