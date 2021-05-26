cmake_minimum_required(VERSION 2.8.3)
project(vafs)

set(CMAKE_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++17 -g -static-libstdc++ -Wno-address-of-packed-member")

find_package(OpenCV REQUIRED)

find_package(
    catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    tf
    cv_bridge
    image_transport
    message_generation
    arducam_usb2_ros
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

#==============================================================================================
#apriltag

set(INC
    ./src
    ./src/lib/apriltag
    ./src/lib/apriltag/common
    ./src/lib/mavlink_v2/common
    ./src/lib/mavlink_v2/ncrl_mavlink
    ./src/camera_dev
    ./src/threads
    ./src/shell
    ./src/apriltag
    ./src/mission
    ./src/common)

set(APRILTAG_SRC
    ./src/lib/apriltag/apriltag.c
    ./src/lib/apriltag/apriltag_pose.c
    ./src/lib/apriltag/apriltag_quad_thresh.c
    ./src/lib/apriltag/common/g2d.c
    ./src/lib/apriltag/common/getopt.c
    ./src/lib/apriltag/common/homography.c
    ./src/lib/apriltag/common/image_u8.c
    ./src/lib/apriltag/common/image_u8x3.c
    ./src/lib/apriltag/common/image_u8x4.c
    ./src/lib/apriltag/common/matd.c
    ./src/lib/apriltag/common/pam.c
    ./src/lib/apriltag/common/pjpeg.c
    ./src/lib/apriltag/common/pjpeg-idct.c
    ./src/lib/apriltag/common/pnm.c
    ./src/lib/apriltag/common/string_util.c
    ./src/lib/apriltag/common/svd22.c
    ./src/lib/apriltag/common/time_util.c
    ./src/lib/apriltag/common/unionfind.c
    ./src/lib/apriltag/common/workerpool.c
    ./src/lib/apriltag/common/zarray.c
    ./src/lib/apriltag/common/zhash.c
    ./src/lib/apriltag/common/zmaxheap.c
    ./src/lib/apriltag/tag16h5.c
    ./src/lib/apriltag/tag25h9.c
    ./src/lib/apriltag/tag36h11.c
    ./src/lib/apriltag/tagCircle21h7.c
    ./src/lib/apriltag/tagCircle49h12.c
    ./src/lib/apriltag/tagCustom48h12.c
    ./src/lib/apriltag/tagStandard41h12.c
    ./src/lib/apriltag/tagStandard52h13.c)

set(SRC
    ${APRILTAG_SRC}
    ./src/apriltag/apriltag_utils.cpp
    ./src/shell/quadshell.cpp
    ./src/shell/shell_cmds.cpp
    ./src/camera_dev/ros_cam.cpp
    ./src/camera_dev/arducam_ros_ctrl.cpp
    ./src/camera_dev/exposure_ctrl.cpp
    ./src/mission/waypoint_mission.cpp
    ./src/mission/trajectory_mission.cpp
    ./src/mission/mission_manager.cpp
    ./src/threads/ros_thread.cpp
    ./src/threads/shell_thread.cpp
    ./src/threads/apriltag_thread.cpp
    ./src/common/sys_time.cpp
    ./src/main.cpp)

add_executable(core ${SRC})
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${INC})
target_link_libraries (core ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(core ${PROJECT_NAME}_generate_messages_cpp)

#==============================================================================================
# intrinsic calibration

SET(CALIBRATION_INC
        ./src
        ./src/calibration
        ./src/camera_dev)

SET(CALIBRATION_SRC
	./src/calibration/intrinsic_calibration.cpp
        ./src/camera_dev/ros_cam.cpp)

add_executable(calibration ${CALIBRATION_SRC})
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${CALIBRATION_INC})
target_link_libraries (calibration ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(calibration ${PROJECT_NAME}_generate_messages_cpp)
