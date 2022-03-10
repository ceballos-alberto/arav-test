#!/usr/bin/env python3

""" ----------------------------------------------------------------------------
    ---------------------- Pub Pose - ARAV Visualizator ------------------------
    ----------------------------------------------------------------------------
    -------------------- Author : Alberto Ceballos Gonzalez --------------------
    -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr --------
    ---------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved ---------
    ---------------------------------------------------------------------------- """

# Import required libraries #
from geometry_msgs.msg import PoseStamped
import rospy

# Topic names #
POSE_TOPIC = "/optitrack/pose"

# Initializing ROS node #
rospy.init_node('pubPose')
rate = rospy.Rate(1)

# ROS publisher #
pub = rospy.Publisher(POSE_TOPIC, PoseStamped, queue_size = 1)
msg = PoseStamped()

# Load message with position & orientation #
msg.pose.position.x = 0.0
msg.pose.position.y = 0.0
msg.pose.position.z = 0.0
msg.pose.orientation.x = 0.0
msg.pose.orientation.y = 0.0
msg.pose.orientation.z = 0.0
msg.pose.orientation.w = 1.0

# Publish messages #
while not rospy.is_shutdown():
    pub.publish(msg)
    rate.sleep()

""" ----------------------------------------------------------------------------
    ---------------------- Pub Pose - ARAV Visualizator ------------------------
    ----------------------------------------------------------------------------
    -------------------- Author : Alberto Ceballos Gonzalez --------------------
    -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr --------
    ---------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved ---------
    ---------------------------------------------------------------------------- """
