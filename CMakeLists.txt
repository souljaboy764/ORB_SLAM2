cmake_minimum_required(VERSION 2.8)
project(ORB_SLAM2)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf
  visualization_msgs
  nav_msgs
  cv_bridge
  message_generation
)

find_package(OpenCV REQUIRED)
#find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_service_files(DIRECTORY srv
  FILES
  PosePointCloud.srv
)

#And now generate the messages
generate_messages(DEPENDENCIES std_msgs sensor_msgs)


catkin_package(CATKIN_DEPENDS roscpp std_msgs sensor_msgs tf visualization_msgs cv_bridge message_runtime)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -w -O3 -g -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w  -O3 -march=native -g -std=c++11")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#MESSAGE("EIGEN PATH " ${EIGEN3_INCLUDE_DIR})


include_directories(
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
#${EIGEN3_INCLUDE_DIR}
${PROJECT_SOURCE_DIR}/Thirdparty/eigen3
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FramePublisher.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Map.cc
src/MapPublisher.cc
src/Optimizer.cc
src/PnPsolver.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/Initializer.cc
src/Publisher.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
#${EIGEN3_LIBS}
${Boost_LIBRARIES}
${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
${catkin_LIBRARIES}
)

# Build examples
add_executable(orb_mono src/ros_mono.cc)
target_link_libraries(orb_mono ${PROJECT_NAME} ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${OpenCV_LIBS}
#${EIGEN3_LIBS}
)

# add_executable(orb_stereo src/ros_stereo.cc)
# target_link_libraries(orb_stereo ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${Boost_LIBRARIES} ${OpenCV_LIBS})

# add_executable(orb_rgbd src/ros_rgbd.cc)
# target_link_libraries(orb_rgbd ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN3_LIBS} ${Boost_LIBRARIES} ${OpenCV_LIBS})

