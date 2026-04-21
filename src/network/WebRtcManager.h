#pragma once

#include <string>
#include <functional>
#include <memory>
#include <QJsonObject>

// Forward declarations
namespace rtc {
    class PeerConnection;
    class DataChannel;
}
class WebSocketSignaling;

class WebRtcManager {
public:
    WebRtcManager();
    ~WebRtcManager();

    // Pass the WebSocketClient to handle signaling (SDP and ICE candidates)
    void initialize(WebSocketSignaling* signalingClient);

    // Callback to alert UI or app when a data channel message arrives
    void setOnMessageCallback(std::function<void(const std::string&)> callback);

    // Send message via DataChannel
    void sendMessage(const std::string& message);

private:
    std::shared_ptr<rtc::PeerConnection> m_peerConnection;
    std::shared_ptr<rtc::DataChannel> m_dataChannel;

    std::function<void(const std::string&)> m_onMessageCallback;
    WebSocketSignaling* m_signalingClient;
};
