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


add_message_files( FILES
  Statistic.msg
  Statistics.msg
)

generate_messages()

catkin_package(
    CATKIN_DEPENDS message_runtime
)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(statistics src/statistics.cpp)
add_dependencies(statistics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(statistics ${catkin_LIBRARIES})
