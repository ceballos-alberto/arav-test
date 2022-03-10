# ---------------------------------------------------------------------------- #
# ---------------------------- ARAV Visualizator ----------------------------- #
# ---------------------------------------------------------------------------- #
# -------------------- Author : Alberto Ceballos Gonzalez -------------------- #
# -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr -------- #
# --------- (c) Copyright 2022. Alberto Ceballos. All Rights Reserved -------- #
# ---------------------------------------------------------------------------- #

cmake_minimum_required (VERSION 3.0.2)
project (visual)
find_package (
  catkin REQUIRED
  rviz
  robot_state_publisher
  std_msgs
  geometry_msgs
  tf2_msgs
  tf2_ros
  xacro
  rospy 
)
include_directories (include ${catkin_INCLUDE_DIRS})

# ---------------------------------------------------------------------------- #
# ---------------------------- ARAV Visualizator ----------------------------- #
# ---------------------------------------------------------------------------- #
# -------------------- Author : Alberto Ceballos Gonzalez -------------------- #
# -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr -------- #
# --------- (c) Copyright 2022. Alberto Ceballos. All Rights Reserved -------- #
# ---------------------------------------------------------------------------- #
