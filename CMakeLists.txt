cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(SDL2 REQUIRED sdl2)
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS} ${SDL2_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(serv_ctrl servos_controller.cpp joystick.cpp)

target_link_libraries(serv_ctrl ${GAZEBO_LIBRARIES} ${SDL2_LIBRARIES})
