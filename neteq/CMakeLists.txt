cmake_minimum_required(VERSION 3.0)
project(webrtc_neteq)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_INSTALL_PREFIX "/home/docker/webrtc_neteq/")

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DNDEBUG -fno-exceptions -fvisibility=hidden -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG -fno-exceptions -fvisibility=hidden -g")

# Include directories
include_directories(
    .
    ../
)

# Source files
file(GLOB SRC *.cc)

# Library target
add_library(plc ${SRC})

install(TARGETS plc
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)