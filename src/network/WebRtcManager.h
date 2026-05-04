#pragma once

#include <string>
#include <functional>
#include <memory>
#include <QJsonObject>
#include <QObject>

// Forward declarations
namespace rtc {
    class PeerConnection;
    class DataChannel;
}

class Signaling;

class WebRtcManager : public QObject {
    Q_OBJECT
public:
    WebRtcManager(std::unique_ptr<Signaling> signaling, QObject *parent = nullptr);
    ~WebRtcManager();

    void connect();

    void disconnect();

    // Callback to alert UI or app when a data channel message arrives
    void setOnMessageCallback(std::function<void(const std::string&)> callback);

    // Send message via DataChannel
    void sendMessage(const std::string& message);

    public slots:
    void publish_message(const std::string& message) {
        sendMessage(message);
    }

private:
    std::unique_ptr<rtc::PeerConnection> m_peer_connection;
    std::shared_ptr<rtc::DataChannel> m_data_channel;

    std::function<void(const std::string&)> m_on_message_callback;

    std::unique_ptr<Signaling> m_signaling;

    signals: 
        void on_connection_state(bool connected);
};
