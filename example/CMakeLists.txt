cmake_minimum_required(VERSION 3.0)
project(example)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_INSTALL_PREFIX "/home/docker/webrtc_neteq/")

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -fvisibility=hidden -DNDEBUG -DWEBRTC_POSIX -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -fvisibility=hidden -DNDEBUG -DWEBRTC_POSIX -g")

# Include and library directories
include_directories(
    .
    ../
)

link_directories(
    .
    ../
)

# Source files
set(SRC_PLC test_plc.cc)
set(SRC_ACCE test_accelerate.cc)
set(SRC_NETEQ test_neteq.cc)
file(GLOB allpcm ${CMAKE_CURRENT_SOURCE_DIR}/*.pcm)
file(COPY ${allpcm}
        DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

# Executable targets
add_executable(AppPlc ${SRC_PLC})
add_executable(AppAcce ${SRC_ACCE})
add_executable(AppNeteq ${SRC_NETEQ})

# Link libraries
target_link_libraries(AppPlc plc webrtc_vad base signal_processing pthread g711)
target_link_libraries(AppAcce plc webrtc_vad base signal_processing pthread)
target_link_libraries(AppNeteq plc webrtc_vad base signal_processing pthread g711)

install(TARGETS AppPlc
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS AppAcce
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS AppNeteq
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)



