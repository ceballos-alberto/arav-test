#!/usr/bin/env python3

""" ----------------------------------------------------------------------------
    -------------------- Tf2 Broadcast - ARAV Visualizator ---------------------
    ----------------------------------------------------------------------------
    -------------------- Author : Alberto Ceballos Gonzalez --------------------
    -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr --------
    ---------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved ---------
    ---------------------------------------------------------------------------- """

# Import required libraries #

import rospy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage

# Topic names #

POSE_TOPIC = "/optitrack/pose"
TF_TOPIC = "/tf"

# Frame names #

fixed_frame = "map"
robot_frame = "base_frame"

# Callback >> executed when a pose is received #

def pose_callback (msg):

    global pub
    global fixed_frame
    global robot_frame

    data = TransformStamped()

    data.header.stamp = rospy.Time.now()
    data.header.frame_id = fixed_frame
    data.child_frame_id = robot_frame

    data.transform.translation.x = msg.pose.position.x
    data.transform.translation.y = msg.pose.position.y
    data.transform.translation.z = msg.pose.position.z
    data.transform.rotation.x = msg.pose.orientation.x
    data.transform.rotation.y = msg.pose.orientation.y
    data.transform.rotation.z = msg.pose.orientation.z
    data.transform.rotation.w = msg.pose.orientation.w

    broadcaster = TFMessage([data])

    pub.publish(broadcaster)

# Initializing ROS node #

rospy.init_node('tf2pub')

# ROS publishers & subscribers #

pub = rospy.Publisher(TF_TOPIC, TFMessage, queue_size = 1)
sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, pose_callback, queue_size = 10)

# Wait for messages #

rospy.spin()

""" ----------------------------------------------------------------------------
    -------------------- Tf2 Broadcast - ARAV Visualizator ---------------------
    ----------------------------------------------------------------------------
    -------------------- Author : Alberto Ceballos Gonzalez --------------------
    -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr --------
    ---------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved ---------
    ---------------------------------------------------------------------------- """
