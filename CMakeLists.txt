cmake_minimum_required(VERSION 3.16)
project(DroneTracking VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(REALSENSE REQUIRED realsense2)

message("header files path of realsense: ${REALSENSE_INCLUDE_DIRS}")
message("library files path of realsense: ${REALSENSE_LIBRARY_DIRS}")

# Automatically enable moc, uic, and rcc for Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Find Qt6 components
find_package(Qt6 COMPONENTS Core Gui Widgets WebSockets REQUIRED)

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

file(GLOB_RECURSE ALL_SOURCE_FILES 
    "src/*.cpp"
    "src/*.c"
)


# Application Executable
add_executable(${PROJECT_NAME} ${ALL_SOURCE_FILES})

message("current directory: ${CMAKE_CURRENT_SOURCE_DIR}")
# Include directories
include_directories(
    ${REALSENSE_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drone_tracking
)
link_directories(
    ${REALSENSE_LIBRARY_DIRS}
)
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/drone_tracking
)


# Target linking
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::WebSockets
    datachannel
    ${REALSENSE_LIBRARIES}
)
