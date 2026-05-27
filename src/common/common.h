#pragma once

#include <chrono>

// #define WEBSOCKET_URL "ws://192.168.88.9:8188"
#define WEBSOCKET_URL "WS://10.51.69.188:8188"

inline long long current_timestamp_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
