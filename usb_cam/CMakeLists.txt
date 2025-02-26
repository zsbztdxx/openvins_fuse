cmake_minimum_required(VERSION 2.8.3)
project(usb_cam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS image_transport roscpp std_msgs std_srvs sensor_msgs camera_info_manager dynamic_reconfigure csjw_msgs cv_bridge)

find_package(LIBUSB10 REQUIRED)
INCLUDE_DIRECTORIES(${LIBUSB10_INCLUDE_DIRS})

## pkg-config libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

## 开优化mjpeg转mono格式无效果(解码ffmpeg库已默认使用neon及优化选项等，所以无效果???)
## 但yuyv转mono效果突出(快7倍(neon),快20倍(native c)，不优化时neon比native c快6倍)
## 开优化后,(传输耗时+解码耗时)720p:,yuyv约50.4ms,mjpeg约22.7ms, 480p:yuyv约16.8ms,mjpeg10.2ms
set(CMAKE_C_FLAGS   "-Wall -O3 -Otime -march=native -ftree-vectorize --vectorize -mfpu=vfpv4 -mfloat-abi=hard -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
set(CMAKE_CXX_FLAGS "-Wall -O3        -march=native -ftree-vectorize -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

generate_dynamic_reconfigure_options(
  cfg/NormalCamera.cfg
)
###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    csjw_msgs
)
###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Build the USB camera library
add_library(${PROJECT_NAME} src/usb_cam.cpp src/isp_dwe_api.cpp src/dvp2usb.cpp)
target_link_libraries(${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${LIBUSB10_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node nodes/usb_cam_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY cfg/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
   FILES_MATCHING PATTERN "*.cfg"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
