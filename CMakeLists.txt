cmake_minimum_required(VERSION 2.8.3)
project(irobotcreate2)

ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  rospy
  std_msgs
  message_generation
  serial
  tf
)
# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Battery.msg
  Brushes.msg
  Bumper.msg
  Buttons.msg
  Diagnostic.msg
  DigitLeds.msg
  IRCharacter.msg
  Leds.msg
  Note.msg
  PlaySong.msg
  RoombaIR.msg
  RoombaSwitch.msg
  ScheduleLeds.msg
  Song.msg
  WheelDrop.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(irobotcreate2_lightweight src/irobotcreate2/OpenInterface.cpp src/irobotcreate2/odometry.cpp src/irobotcreate2_lightweight.cpp)
add_dependencies(irobotcreate2_lightweight irobotcreate2_lightweight_gencpp)

add_executable(irobotcreate2 src/irobotcreate2/OpenInterface.cpp src/irobotcreate2/odometry.cpp src/irobotcreate2.cpp)
add_dependencies(irobotcreate2 irobotcreate2_gencpp)

add_executable(joy_node src/joy_node.cpp)
add_dependencies(joy_node irobotcreate2_generate_messages_cpp)

add_executable(keyboard_node src/keyboard_node.cpp)
add_dependencies(keyboard_node irobotcreate2_generate_messages_cpp)

add_executable(basic_commands src/irobotcreate2/OpenInterface.cpp src/irobotcreate2/odometry.cpp src/basic_commands.cpp)

target_link_libraries(irobotcreate2_lightweight ${catkin_LIBRARIES})

target_link_libraries(irobotcreate2 ${catkin_LIBRARIES})

target_link_libraries(basic_commands ${catkin_LIBRARIES})

target_link_libraries(joy_node ${catkin_LIBRARIES})

target_link_libraries(keyboard_node ${catkin_LIBRARIES})