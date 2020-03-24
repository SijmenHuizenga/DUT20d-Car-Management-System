cmake_minimum_required(VERSION 2.8.3)
project(cms)

add_compile_options(-std=c++11 -Werror -Wall -fstack-protector)


find_package(catkin REQUIRED COMPONENTS
             cmake_modules
             rospy
             roscpp
             message_generation
             message_runtime
             topic_tools
             std_msgs)


add_service_files( FILES
  RecordingUpdateConfig.srv
  RecordingGetBagInfo.srv
)

add_message_files( FILES
  Statistic.msg
  Statistics.msg
  RecordingStatus.msg
  RecordingConfig.msg
  BagTopicInfo.msg
  PingResult.msg
)

generate_messages()

catkin_package(
    CATKIN_DEPENDS message_runtime
)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(statistics src/statisticsnode.cpp)
add_dependencies(statistics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(statistics ${catkin_LIBRARIES})

add_executable(pinger src/pingnode.cpp)
add_dependencies(pinger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pinger ${catkin_LIBRARIES})
