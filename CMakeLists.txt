cmake_minimum_required(VERSION 3.8.2)
project(hexopods)

get_board_id(board_id uno atmega328)

add_arduino_executable(hexopods ${board_id} hexopods.cpp)
upload_arduino_target(hexopods "${board_id}" COM3)

add_arduino_library (Multiservo_lib ${board_id} library/Multiservo/Multiservo.cpp)
target_include_directories(Multiservo_lib PUBLIC library/Multiservo)
link_arduino_library(hexopods Multiservo_lib ${board_id})