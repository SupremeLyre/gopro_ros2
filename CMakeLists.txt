cmake_minimum_required(VERSION 3.10)
project(gopro_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ffmpeg REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 3 REQUIRED)

if(FFMPEG_FOUND)
    message("Found FFMPEG/LibAV libraries")
else()
    message("Can't find libavcodec, libavformat or libavutil. Add them!")
endif()

include_directories(
    include 
    ${FFMPEG_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )


file(GLOB c_src "src/*.c")
file(GLOB cpp_src "src/*.cpp")

add_library(${PROJECT_NAME} SHARED ${c_src} ${cpp_src})
target_link_libraries(${PROJECT_NAME} ${FFMPEG_LIBRARIES} ${OpenCV_LIBS})
ament_target_dependencies(${PROJECT_NAME}
  builtin_interfaces
  cv_bridge
  geometry_msgs
  rclcpp
  rosbag2_cpp
  sensor_msgs
  std_msgs
)

add_executable(gopro_to_asl gopro_to_asl.cpp)
target_link_libraries(gopro_to_asl ${PROJECT_NAME})
ament_target_dependencies(gopro_to_asl rclcpp)

add_executable(gopro_to_rosbag gopro_to_rosbag.cpp)
target_link_libraries(gopro_to_rosbag ${PROJECT_NAME})
ament_target_dependencies(gopro_to_rosbag
  builtin_interfaces
  rclcpp
  rosbag2_cpp
  sensor_msgs
  std_msgs
)

install(TARGETS ${PROJECT_NAME} gopro_to_asl gopro_to_rosbag
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/asl_to_bag.py
  scripts/bag_to_asl.py
  scripts/orbtimestamp.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  builtin_interfaces
  cv_bridge
  geometry_msgs
  rclcpp
  rosbag2_cpp
  sensor_msgs
  std_msgs
)

ament_package()
