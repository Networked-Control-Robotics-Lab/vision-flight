cmake_minimum_required(VERSION 2.8.3)
project(apriltag)

set(CMAKE_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-std=c++17 -g -static-libstdc++ -Wno-address-of-packed-member")

find_package(PythonLibs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(
    catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    tf
    message_generation
)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

set(INC
    ./src
    ./src/lib/apriltag/
    ./src/lib/apriltag/common)

set(SRC
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
    ./src/lib/apriltag/tagStandard52h13.c
    ./src/main.cpp)

add_executable(apriltag ${SRC})
include_directories(${catkin_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${INC})
target_link_libraries (apriltag ${PYTHON_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(apriltag ${PROJECT_NAME}_generate_messages_cpp)
