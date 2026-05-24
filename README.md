# DroneTracking

A cross-platform (Windows & Linux) C++ Desktop Application utilizing Qt6, WebSockets, and WebRTC to communicate with a remote server/drone.

## Project Structure
- `src/` - Contains all C++ source code.
  - `test_main.cpp` - Test application entry point, for debug and experiment of code.
  - `camera_calibration_main.cpp` - Camera calibration application entry point.
  - `drone_tracking_main.cpp` - Drone tracking application entry point.
  - `photo_capturer_main.cpp` - Photo capturer application entry point.
  - `camera/` - Camera module including generic and realsense cameras.
  - `gui/` - Qt UI Classes (`MainWindow` implemented using layout managers).
  - `network/` - Network handling (`WebSocketClient` for signaling, and `WebRtcManager` using libdatachannel).
  - `tracking/` - ArUco markers based tracking and related configuration.
- `CMakeLists.txt` - Modern CMake script using FetchContent to automatically bring in the WebRTC dependency without requiring manual installation.

## Technologies Used
- **Qt6** (Core, Gui, Widgets, WebSockets) - UI rendering, signaling WebSocket client.
- **libdatachannel** - A lightweight C/C++ WebRTC network library, chosen over the main Google WebRTC implementation for simplicity and cross-platform compatibility without heavy build-system modifications.- **CMake** (v3.16+)

## Setup & Building

### Prerequisites
- CMake >= 3.16
- A modern C++17 compiler (GCC/Clang/MSVC)
- Qt 6 (install via Qt Maintenance Tool or your system's package manager)

### Build Instructions
```bash
# From the project root directory
mkdir build
cd build
cmake ..
cmake --build .
```
*(On Windows, you may need to open the project via CMake-GUI or QtCreator, or run using Developer Command Prompt with Ninja or Visual Studio generators.)*