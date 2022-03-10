#!/usr/bin/env python3

""" ------------------------------------------------------------------------
    ----------------------- Image Publisher (Test) -------------------------
    ------------------------------------------------------------------------
    ------------------ Author : Alberto Ceballos Gonzalez ------------------
    ------ E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr ------
    -------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved -------
    ------------------------------------------------------------------------ """

# Import required libraries #
from sensor_msgs.msg import Image
import cv_bridge
import cv2
import rospy
import sys

# Topic names #
TOPIC_RIGHT = "/data/images/right"
TOPIC_LEFT = "/data/images/left"

# Constants #
UPDATE_RATE = 1

# Paths #
base_path = sys.argv[1]
right_path = base_path + "/utils/test_right.jpg"
left_path = base_path + "/utils/test_left.jpg"

# Load images #
imageRight = cv2.imread(right_path)
imageLeft = cv2.imread(left_path)

# Initializing ROS node #
rospy.init_node('pub_images')
rate = rospy.Rate(UPDATE_RATE)

# ROS Publishers #
pubRight = rospy.Publisher(TOPIC_RIGHT, Image, queue_size=1)
pubLeft = rospy.Publisher(TOPIC_LEFT, Image, queue_size=1)

# OpenCV bridges #
bridgeRight = cv_bridge.CvBridge()
bridgeLeft = cv_bridge.CvBridge()

# Main loop #
while not rospy.is_shutdown():
    pubRight.publish(bridgeRight.cv2_to_imgmsg(imageRight, encoding="rgb8"))
    pubLeft.publish(bridgeLeft.cv2_to_imgmsg(imageLeft, encoding="rgb8"))
    rate.sleep ()

""" ------------------------------------------------------------------------
    ----------------------- Image Publisher (Test) -------------------------
    ------------------------------------------------------------------------
    ------------------ Author : Alberto Ceballos Gonzalez ------------------
    ------ E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr ------
    -------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved -------
    ------------------------------------------------------------------------ """
