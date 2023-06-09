cmake_minimum_required(VERSION 3.0)
project(base)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_INSTALL_PREFIX "/home/docker/webrtc_neteq")

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-exceptions -fvisibility=hidden -DNDEBUG -DWEBRTC_POSIX -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions -fvisibility=hidden -DNDEBUG -DWEBRTC_POSIX -g")

# Include directories
include_directories(
    .
)

# Source files
file(GLOB DSPSRC common_audio/signal_processing/*.c)
list(APPEND DSPSRC common_audio/third_party/spl_sqrt_floor/spl_sqrt_floor.c)
file(GLOB BASESRC rtc_base/*.cc)
file(GLOB VADSRC common_audio/vad/*.c)
set(SCALESRC common_audio/signal_processing/dot_product_with_scale.cc)

# Library targets
add_library(signal_processing ${DSPSRC} ${SCALESRC})
add_library(base ${BASESRC})
add_library(webrtc_vad ${VADSRC})

install(TARGETS signal_processing
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS base
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS webrtc_vad
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# External libraries
add_subdirectory(neteq)
add_subdirectory(codec/interface)


# Example target
add_subdirectory(example)

# Clean target
add_custom_target(clean-all
    COMMAND ${CMAKE_BUILD_TOOL} clean
    COMMAND rm -rf ${CMAKE_BINARY_DIR}/*.a
)