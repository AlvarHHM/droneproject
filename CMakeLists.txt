cmake_minimum_required(VERSION 2.8.3)
project(droneproject)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
set(CMAKE_CXX_FLAGS "-std=c++0x")
find_package(OpenCV)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  geometry_msgs
  ardrone_autonomy
  cv_bridge
  image_transport
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS} mftracker tld cvblobs)

## Declare a cpp library
# add_library(droneproject
#   src/${PROJECT_NAME}/droneproject.cpp
# )

## Declare a cpp executable
# add_executable(droneproject_node src/droneproject_node.cpp)
add_executable(DroneProject 
	src/Main.cpp 
	src/Selector.h src/Selector.cpp 
	src/NavigationData.h src/NavigationData.cpp 
	src/ImageProcess.h src/ImageProcess.cpp 
	src/State.h src/State.cpp 
	src/CameraImageProcess.h src/CameraImageProcess.cpp 
	src/VideoImageProcess.h src/VideoImageProcess.cpp 
	src/StateData.h src/StateData.cpp 
	src/Flight.h src/Flight.cpp 
	src/Pid.h src/Pid.cpp
    src/tld/Clustering.h
	src/tld/Clustering.cpp
    src/tld/DetectionResult.h
    src/tld/DetectionResult.cpp
    src/tld/DetectorCascade.h
    src/tld/DetectorCascade.cpp
    src/tld/EnsembleClassifier.h
    src/tld/EnsembleClassifier.cpp
    src/tld/ForegroundDetector.h
    src/tld/ForegroundDetector.cpp
    src/tld/MedianFlowTracker.h
    src/tld/MedianFlowTracker.cpp
    src/tld/NNClassifier.h
    src/tld/NNClassifier.cpp
    src/tld/TLD.h
    src/tld/TLD.cpp
    src/tld/TLDUtil.h
    src/tld/TLDUtil.cpp
    src/tld/VarianceFilter.h
    src/tld/VarianceFilter.cpp
    src/tld/IntegralImage.h
    src/tld/NormalizedPatch.h
    src/mftracker/BB.h
    src/mftracker/BB.cpp
    src/mftracker/BBPredict.h
    src/mftracker/BBPredict.cpp
    src/mftracker/FBTrack.h
    src/mftracker/FBTrack.cpp
    src/mftracker/Lk.h
    src/mftracker/Lk.cpp
    src/mftracker/Median.h
    src/mftracker/Median.cpp
    src/cvblobs/blob.cpp
    src/cvblobs/blob.h
    src/cvblobs/BlobContour.cpp
    src/cvblobs/BlobContour.h
    src/cvblobs/BlobLibraryConfiguration.h
    src/cvblobs/BlobOperators.cpp
    src/cvblobs/BlobOperators.h
    src/cvblobs/BlobProperties.cpp
    src/cvblobs/BlobProperties.h
    src/cvblobs/BlobResult.cpp
    src/cvblobs/BlobResult.h
    src/cvblobs/ComponentLabeling.cpp
    src/cvblobs/ComponentLabeling.h
	)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(droneproject_node droneproject_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(DroneProject
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS droneproject droneproject_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#