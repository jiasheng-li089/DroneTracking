#include "WebRtcManager.h"
#include "WebSocketSignaling.h"
#include "logger.h"

#include <rtc/rtc.hpp>
#include <QJsonDocument>

WebRtcManager::WebRtcManager(std::unique_ptr<Signaling> signaling, QObject *parent) : m_signaling(std::move(signaling)), QObject(parent) {}

void WebRtcManager::connect() {
    rtc::Configuration config;
    // Public Google STUN server for NAT traversal
    config.iceServers.emplace_back(rtc::IceServer{"stun:stun.l.google.com:19302"});

    m_peer_connection = std::make_unique<rtc::PeerConnection>(config);

    m_peer_connection->onStateChange([this](rtc::PeerConnection::State state) {
        auto state_str = [state]() {
            switch (state) {
                case rtc::PeerConnection::State::New: return "New";
                case rtc::PeerConnection::State::Connecting: return "Connecting";
                case rtc::PeerConnection::State::Connected: return "Connected";
                case rtc::PeerConnection::State::Disconnected: return "Disconnected";
                case rtc::PeerConnection::State::Failed: return "Failed";
                case rtc::PeerConnection::State::Closed: return "Closed";
                default: return "Unknown";
            }
        }();
        spdlog::info("WebRTC State {}: {}", static_cast<int>(state), state_str);

        if (state == rtc::PeerConnection::State::Connected) {
            spdlog::info("Peer connection established");
            emit on_connection_state(true);
        } else if (state == rtc::PeerConnection::State::Failed || state == rtc::PeerConnection::State::Closed) {
            spdlog::warn("Peer connection failed or closed");
            emit on_connection_state(false);
        }
    });

    m_peer_connection->onGatheringStateChange([](rtc::PeerConnection::GatheringState state) {
        auto state_str = [state]() {
            switch (state) {
                case rtc::PeerConnection::GatheringState::New: return "New";
                case rtc::PeerConnection::GatheringState::InProgress: return "InProgress";
                case rtc::PeerConnection::GatheringState::Complete: return "Complete";
                default: return "Unknown";
            }
        }();
        spdlog::info("ICE Gathering State: {}: {}", int(state), state_str);
    });

    // Handle Local ICE Candidates
    m_peer_connection->onLocalCandidate([this](rtc::Candidate candidate) {
        spdlog::debug("New local ICE candidate generated.");
        if (m_signaling) {
            // Normally you would serialize this to JSON and send it over WebSocket
            // Example:
            QJsonObject json;
            json["type"] = "candidate";
            json["candidate"] = QString::fromStdString(candidate.candidate());
            json["sdpMid"] = QString::fromStdString(candidate.mid());
        }
    });

    // Handle incoming local description (SDP Offer/Answer)
    m_peer_connection->onLocalDescription([this](rtc::Description description) {
        auto sdp = std::string(description);
        auto type = description.typeString();
        spdlog::debug("Local SDP Description generated, type: {}, SDP: {}", type, sdp);
        if (m_signaling) {
            auto answer = m_signaling->exchange_offer(sdp);

            if (answer.empty()) {
                spdlog::error("Failed to exchange offer with remote server");
                return;
            }
            spdlog::debug("Received remote SDP answer: {}", answer);
            m_peer_connection->setRemoteDescription(rtc::Description(answer, "answer"));
        }
    });

    // Create a data channel
    m_data_channel = m_peer_connection->createDataChannel("drone_data");
    m_data_channel->onOpen([this]() {
        spdlog::info("Data channel opened");
        m_data_channel->send("Hello from DroneTracking WebRTC!");
    });

    m_data_channel->onMessage([this](std::variant<rtc::binary, rtc::string> message) {
        if (std::holds_alternative<rtc::string>(message)) {
            std::string msg = std::get<rtc::string>(message);
            spdlog::debug("Received msg on data channel: {}", msg);
            if (m_on_message_callback) {
                m_on_message_callback(msg);
            }
        }
    });
}

WebRtcManager::~WebRtcManager() {
    disconnect();
}

void WebRtcManager::disconnect() {
    if (m_data_channel) {
        m_data_channel->close();
        m_data_channel.reset();
    }
    if (m_peer_connection) {
        m_peer_connection->close();
        m_peer_connection.reset();
    }
    if (m_signaling) {
        m_signaling->end();
    }
}

void WebRtcManager::setOnMessageCallback(std::function<void(const std::string&)> callback) {
    m_on_message_callback = std::move(callback);
}

void WebRtcManager::sendMessage(const std::string& message) {
    if (m_data_channel && m_data_channel->isOpen()) {
        // spdlog::debug("Sending message on data channel: {}", message);
        m_data_channel->send(message);
    } else {
        spdlog::error("Cannot send message, data channel is invalid or not open: {}", message);
    }
}
