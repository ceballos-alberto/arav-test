# ---------------------------------------------------------------------------- #
# ---------------- Environment Perception Module (EPM) - ARAV ---------------- #
# ---------------------------------------------------------------------------- #
# -------------------- Author : Alberto Ceballos Gonzalez -------------------- #
# -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr -------- #
# --------- (c) Copyright 2021. Alberto Ceballos. All Rights Reserved -------- #
# ---------------------------------------------------------------------------- #

cmake_minimum_required (VERSION 3.0.2)
project (perception)
find_package (
  catkin REQUIRED
  rviz
  sensor_msgs
  std_msgs
  image_transport
  cv_bridge
  roscpp
  rospy 
)
find_package (OpenCV REQUIRED)
find_package(PCL REQUIRED)
SET (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/code)
include_directories (include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable (depth code/depth.cpp)
target_link_libraries (depth ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

# ---------------------------------------------------------------------------- #
# ---------------- Environment Perception Module (EPM) - ARAV ---------------- #
# ---------------------------------------------------------------------------- #
# -------------------- Author : Alberto Ceballos Gonzalez -------------------- #
# -------- E-mail : alberto.ceballos-gonzalez@student.isae-supaero.fr -------- #
# --------- (c) Copyright 2021. Alberto Ceballos. All Rights Reserved -------- #
# ---------------------------------------------------------------------------- #		
