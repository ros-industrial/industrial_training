#!/usr/bin/env python
import rospy
import sys
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def rotateImg(img, angle):
    rows,cols,ch = img.shape
    M = cv2.getRotationMatrix2D((cols/2,rows/2),angle,1)
    return cv2.warpAffine(img,M,(cols,rows))

def start_node(filename):
    rospy.init_node('image_pub')
    rospy.loginfo('image_pub node started')

    img = cv2.imread(filename)
    # cv2.imshow("image", img)
    # cv2.waitKey(2000)

    bridge = CvBridge()
    imgMsg = bridge.cv2_to_imgmsg(img, "bgr8")

    pub = rospy.Publisher('image', Image, queue_size=10)

    angle = 0
    while not rospy.is_shutdown():
        rotImg = rotateImg(img, angle)
        imgMsg = bridge.cv2_to_imgmsg(rotImg, "bgr8")

        pub.publish(imgMsg)
        angle = (angle + 10) % 360
        rospy.Rate(1.0).sleep()  # 1 Hz

if __name__ == '__main__':
    try:
        start_node( rospy.myargv(argv=sys.argv)[1] )
    except rospy.ROSInterruptException:
        pass
