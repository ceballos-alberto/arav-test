#!/usr/bin/env python3

""" ----------------------------------------------------------------------------
    -------------------------- OptiTrack Transformer ---------------------------
    ----------------------------------------------------------------------------
    -------------------- Author : Alberto Ceballos Gonzalez --------------------
    -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr --------
    ---------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved ---------
    ---------------------------------------------------------------------------- """

# Import required libraries #
from geometry_msgs.msg import PoseStamped
import rospy

# Topic names #
IN_TOPIC = "/vrpn_client_node/ARAV/pose"
OUT_TOPIC = "/optitrack/pose"

# Offsets #
xOffset = 2.0

# Callback >> executed when a pose is received #
def callback (msgIn):
    msgOut = PoseStamped()
    msgOut.pose.position.x = msgIn.pose.position.x + xOffset
    msgOut.pose.position.y = msgIn.pose.position.y
    msgOut.pose.position.z = msgIn.pose.position.z
    msgOut.pose.orientation.x = msgIn.pose.orientation.x
    msgOut.pose.orientation.y = msgIn.pose.orientation.y
    msgOut.pose.orientation.z = msgIn.pose.orientation.z
    msgOut.pose.orientation.w = msgIn.pose.orientation.w
    pub.publish(msgOut)

# Initializing ROS node #
rospy.init_node('optiTrans')

# ROS publishers & subscribers #
pub = rospy.Publisher(OUT_TOPIC, PoseStamped, queue_size = 1)
sub = rospy.Subscriber(IN_TOPIC, PoseStamped, callback, queue_size = 1)

# Wait for messages #
rospy.spin()

""" ----------------------------------------------------------------------------
    -------------------------- OptiTrack Transformer ---------------------------
    ----------------------------------------------------------------------------
    -------------------- Author : Alberto Ceballos Gonzalez --------------------
    -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr --------
    ---------- (c) Copyright 2022 Alberto Ceballos All Rights Reserved ---------
    ---------------------------------------------------------------------------- """
