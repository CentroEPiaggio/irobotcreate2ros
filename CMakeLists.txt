cmake_minimum_required(VERSION 2.8.3)
project(irobotcreate2)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  rospy
  std_msgs
  message_generation
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

include_directories(include)

add_executable(roomba500 src/iRobot/OpenInterface.cpp src/roomba500_lightweight.cpp)
add_executable(roomba560 src/iRobot/OpenInterface.cpp src/roomba560.cpp)

target_link_libraries(roomba500 ${catkin_LIBRARIES})

target_link_libraries(roomba560 ${catkin_LIBRARIES})