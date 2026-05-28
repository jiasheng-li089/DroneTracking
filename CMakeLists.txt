cmake_minimum_required(VERSION 3.20)
project(DroneTracking VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(DEBUG_CHANNEL "Enable debug webrtc data channel" ON)
option(SHOW_SINGLE_MARKER_POSE "Show the pose of a single marker for debugging" ON)
option(ENABLE_STEREO_CALIBRATION "Enable stereo calibration mode" OFF)
option(CALIBRATE_ONCE "Calibrate the camera once and then use the cached pose" ON)
option(USE_REALSENSE_CAMERA "Use the realsense cameras" ON)
option(ENABLE_VIDEO_UPDATE "If show video updates on GUI during tracking" ON)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    message(STATUS "Found ccache: ${CCACHE_PROGRAM}")
else()
    message(STATUS "ccache not found. Compilation will not be accelerated.")
endif()
find_package(PkgConfig REQUIRED)

if (USE_REALSENSE_CAMERA)
    pkg_check_modules(REALSENSE QUIET realsense2)

    if (NOT REALSENSE_FOUND)
        message(WARNING "cannot find the dependency: realsense")
        set(USE_REALSENSE_CAMERA OFF)
        SET(ENABLE_DEPTH OFF)
    endif()

endif()


# Automatically enable moc, uic, and rcc for Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Find Qt6 components
find_package(Qt6 COMPONENTS Core Gui Widgets WebSockets OpenGLWidgets REQUIRED)

# Find the opencv components
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d objdetect videoio video highgui)

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

file(GLOB_RECURSE ALL_SOURCE_FILES
    "src/*.cpp"
    "src/*.c"
    "src/*.h"
    "src/*.hpp"
)

set(LINK_DEPENDENCY
    datachannel
    spdlog::spdlog
)

list(FILTER ALL_SOURCE_FILES EXCLUDE REGEX "src/.*_main\.cpp$")

if (NOT USE_REALSENSE_CAMERA)
    # remove source files related to realsense camera from the file list
    list(FILTER ALL_SOURCE_FILES EXCLUDE REGEX "src/.*RealSense.*\.(h|cpp)$")
    message(STATUS "current all source files: ${ALL_SOURCE_FILES}")
else()
    include_directories(${REALSENSE_INCLUDE_DIRS})
    link_directories(${REALSENSE_LIBRARY_DIRS})

    list(APPEND LINK_DEPENDENCY ${REALSENSE_LIBRARIES})
endif()

if (NOT Qt6_FOUND)
    message(WARNING "Cannot find the dependency: Qt6")
else()
    list(APPEND LINK_DEPENDENCY
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
        Qt6::WebSockets
        Qt6::OpenGLWidgets
    )
endif()

if (NOT OpenCV_FOUND)
    message(WARNING "Cannot find the dependency: OpenCV")
else()
    list(APPEND LINK_DEPENDENCY ${OpenCV_LIBS})
endif()

message(STATUS "dependency list: ${LINK_DEPENDENCY}")

set(TARGET_NAMES
    "PhotoCapturer"
    "CameraCalibration"
    "DroneTracking"
    "Test"
    "Demo"
)
set(TARGET_MAINS
    "photo_capturer_main.cpp"
    "camera_calibration_main.cpp"
    "drone_tracking_main.cpp"
    "test_main.cpp"
    "demo_main.cpp"
)

# Application Executable
foreach(key value IN ZIP_LISTS TARGET_NAMES TARGET_MAINS)
    message(STATUS "adding target: ${key} with main file: ${value}")
    add_executable("${key}" ${ALL_SOURCE_FILES} ${CMAKE_CURRENT_SOURCE_DIR}/src/${value})
endforeach()

configure_file(
  ${CMAKE_SOURCE_DIR}/src/config.h.in
  ${CMAKE_BINARY_DIR}/config.h
)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Targets linking
foreach(key value IN ZIP_LISTS TARGET_NAMES TARGET_MAINS)
    target_include_directories("${key}" PRIVATE ${CMAKE_BINARY_DIR})

    target_link_libraries("${key}" PRIVATE
        ${LINK_DEPENDENCY}
    )
endforeach()
