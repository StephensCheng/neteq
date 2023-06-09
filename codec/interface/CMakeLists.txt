cmake_minimum_required(VERSION 3.0)
project(codec_interface)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_INSTALL_PREFIX "/home/docker/webrtc_neteq/")

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DNDEBUG -DWEBRTC_POSIX -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG -DWEBRTC_POSIX -g")

# Include directories
include_directories(
    .
    ./g711
    ./g722
)

# Source files for g711
file(GLOB G711SRC g711/*.c)
add_library(g711 ${G711SRC})
install(TARGETS g711
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# Source files for g722
file(GLOB G722SRC g722/*.c)
add_library(g722 ${G722SRC})
install(TARGETS g722
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

