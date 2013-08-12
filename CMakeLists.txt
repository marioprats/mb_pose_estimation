cmake_minimum_required(VERSION 2.8.3)
project(mb_pose_estimation)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  lima
  message_generation
  roscpp
)
find_package(VISP REQUIRED)
find_package(Boost REQUIRED thread program_options system filesystem)

add_action_files(FILES MBTrack.action)
add_service_files(FILES PoseEstimation.srv)

generate_messages(DEPENDENCIES std_msgs geometry_msgs actionlib_msgs)

catkin_package(
  INCLUDE_DIRS
    include  
  LIBRARIES
    mb_pose_estimation 
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    lima
    message_runtime
    roscpp
  DEPENDS VISP
)

add_definitions(${VISP_DEFINITIONS})
link_directories(${VISP_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${VISP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/mb_pose_estimation.cpp src/utils.cpp)

add_executable(pose_estimation_service src/pose_estimation_service.cpp)
target_link_libraries(pose_estimation_service ${PROJECT_NAME} ${catkin_LIBRARIES} ${VISP_LIBRARIES} ${Boost_LIBRARIES})

add_executable(pose_estimation_client src/pose_estimation_client.cpp)
target_link_libraries(pose_estimation_client ${PROJECT_NAME} ${catkin_LIBRARIES} ${VISP_LIBRARIES} ${Boost_LIBRARIES})

add_executable(training_node src/training_node.cpp)
target_link_libraries(training_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${VISP_LIBRARIES} ${Boost_LIBRARIES})

add_executable(track_action_server src/track_action_server.cpp)
target_link_libraries(track_action_server ${PROJECT_NAME} ${catkin_LIBRARIES} ${VISP_LIBRARIES} ${Boost_LIBRARIES})

add_executable(track_action_client src/track_action_client.cpp)
target_link_libraries(track_action_client ${PROJECT_NAME} ${catkin_LIBRARIES} ${VISP_LIBRARIES} ${Boost_LIBRARIES})

install(
  TARGETS
    ${PROJECT_NAME}
    pose_estimation_service
    pose_estimation_client
    track_action_server
    track_action_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

