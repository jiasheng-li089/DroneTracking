cmake_minimum_required(VERSION 3.20)
project(DroneTracking VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(DEBUG_CHANNEL "Enable debug webrtc data channel" OFF)
option(ENABLE_DEPTH "Enable the depth stream for tracking" OFF)
option(SHOW_SINGLE_MARKER_POSE "Show the pose of a single marker for debugging" ON)
option(ENABLE_STEREO_CALIBRATION "Enable stereo calibration mode" OFF)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    message(STATUS "Found ccache: ${CCACHE_PROGRAM}")
else()
    message(STATUS "ccache not found. Compilation will not be accelerated.")
endif()
find_package(PkgConfig REQUIRED)
pkg_check_modules(REALSENSE REQUIRED realsense2)

message("header files path of realsense: ${REALSENSE_INCLUDE_DIRS}")
message("library files path of realsense: ${REALSENSE_LIBRARY_DIRS}")

# Automatically enable moc, uic, and rcc for Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Find Qt6 components
find_package(Qt6 COMPONENTS Core Gui Widgets WebSockets OpenGLWidgets REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d objdetect)

# LibDataChannel for WebRTC
# We use FetchContent to integrate a lightweight C/C++ WebRTC library,
# which avoids the complexity of building Google's large WebRTC codebase.
include(FetchContent)
FetchContent_Declare(
    libdatachannel
    GIT_REPOSITORY https://github.com/paullouisageneau/libdatachannel.git
    GIT_TAG        v0.24.2
)
# Disable optional components to speed up the sub-build
set(NO_WEBSOCKET ON CACHE BOOL "Disable libdatachannel internal websocket" FORCE)
set(NO_EXAMPLES ON CACHE BOOL "Disable examples" FORCE)
set(NO_TESTS ON CACHE BOOL "Disable tests" FORCE)
FetchContent_MakeAvailable(libdatachannel)

FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.15.1
)
FetchContent_MakeAvailable(spdlog)

FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG        master
)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE INTERNAL "")
FetchContent_MakeAvailable(yaml-cpp)

file(GLOB_RECURSE ALL_SOURCE_FILES
    "src/*.cpp"
    "src/*.c"
    "src/*.h"
    "src/*.hpp"
)

list(FILTER ALL_SOURCE_FILES EXCLUDE REGEX "src/.*_main\.cpp$")

message("All source files: ${ALL_SOURCE_FILES}")

set(TARGET_NAMES 
    "PhotoCapturer" 
    "Test" 
    "CameraCalibration" 
    "DroneTracking"
)
set(TARGET_MAINS 
    "photo_capturer_main.cpp" 
    "test_realsense_main.cpp" 
    "camera_calibration_main.cpp" 
    "drone_tracking_main.cpp"
)

# Application Executable
foreach(key value IN ZIP_LISTS TARGET_NAMES TARGET_MAINS)
    message("adding target: ${key} with main file: ${value}")
    add_executable("${PROJECT_NAME}_${key}" ${ALL_SOURCE_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/src/${value})
endforeach()


# Include directories
include_directories(
    ${REALSENSE_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
link_directories(
    ${REALSENSE_LIBRARY_DIRS}
)

# Targets linking
foreach(key value IN ZIP_LISTS TARGET_NAMES TARGET_MAINS)
    if(DEBUG_CHANNEL)
        target_compile_definitions("${PROJECT_NAME}_${key}" PRIVATE DEBUG_CHANNEL)
    endif()
    if(ENABLE_DEPTH)
        target_compile_definitions("${PROJECT_NAME}_${key}" PRIVATE ENABLE_DEPTH_CAMERA)
    endif()
    if (SHOW_SINGLE_MARKER_POSE)
        target_compile_definitions("${PROJECT_NAME}_${key}" PRIVATE SHOW_SINGLE_MARKER_POSE)
    endif()
    if (ENABLE_STEREO_CALIBRATION)
        target_compile_definitions("${PROJECT_NAME}_${key}" PRIVATE ENABLE_STEREO_CALIBRATION)
    endif()
    target_link_libraries("${PROJECT_NAME}_${key}" PRIVATE
            Qt6::Core
            Qt6::Gui
            Qt6::Widgets
            Qt6::WebSockets
            Qt6::OpenGLWidgets
            datachannel
            spdlog::spdlog
            yaml-cpp::yaml-cpp
        ${REALSENSE_LIBRARIES}
        ${OpenCV_LIBS}
    )
endforeach()
