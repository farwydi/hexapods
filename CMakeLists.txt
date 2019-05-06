cmake_minimum_required(VERSION 3.8.2)
project(hexopods)

# Call a framework utility function, passing it information about the hardware board that will be used - This function returns a structure known only to the framework
get_board_id(board_id uno atmega328)

# Create an executable suitable for Arduino using CMake-style target-creation
add_arduino_executable(hexopods ${board_id} hexopods.cpp)
# Upload the created target through a connected Serial Port (Where your board is connected to)
upload_arduino_target(hexopods "${board_id}" COM3)

#add_arduino_library(twi_lib STATIC ${twi_lib_path}/twi.c)
#target_include_directories(twi_lib PUBLIC ${twi_lib_path})
#link_arduino_library(hexopods twi_lib ${board_id})

#find_arduino_library(Multiservo Multiservo 3RD_PARTY)
#link_arduino_library(hexopods Multiservo)

#add_library(wire_lib STATIC library/Wire/Wire.cpp)
#target_include_directories(wire_lib PUBLIC library/Wire)
#link_arduino_library(hexopods wire_lib ${board_id})

add_arduino_library (Multiservo_lib ${board_id} library/Multiservo/Multiservo.cpp)
target_include_directories(Multiservo_lib PUBLIC library/Multiservo)
link_arduino_library(hexopods Multiservo_lib ${board_id})