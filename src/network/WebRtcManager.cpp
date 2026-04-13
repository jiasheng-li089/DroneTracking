#include "WebRtcManager.h"
#include "WebSocketClient.h"
#include <rtc/rtc.hpp>
#include <iostream>
#include <QJsonDocument>

WebRtcManager::WebRtcManager() : m_signalingClient(nullptr) {}

WebRtcManager::~WebRtcManager() {
    if (m_peerConnection) {
        m_peerConnection->close();
    }
}

void WebRtcManager::initialize(WebSocketClient* signalingClient) {
    m_signalingClient = signalingClient;

    rtc::Configuration config;
    // Public Google STUN server for NAT traversal
    config.iceServers.emplace_back(rtc::IceServer{"stun:stun.l.google.com:19302"});

    m_peerConnection = std::make_shared<rtc::PeerConnection>(config);

    m_peerConnection->onStateChange([](rtc::PeerConnection::State state) {
        std::cout << "WebRTC State: " << state << std::endl;
    });

    m_peerConnection->onGatheringStateChange([](rtc::PeerConnection::GatheringState state) {
        std::cout << "ICE Gathering State: " << state << std::endl;
    });

    // Handle Local ICE Candidates
    m_peerConnection->onLocalCandidate([this](rtc::Candidate candidate) {
        std::cout << "New local ICE candidate generated." << std::endl;
        if (m_signalingClient) {
            // Normally you would serialize this to JSON and send it over WebSocket
            // Example:
            QJsonObject json;
            json["type"] = "candidate";
            json["candidate"] = QString::fromStdString(candidate.candidate());
            json["sdpMid"] = QString::fromStdString(candidate.mid());
            // m_signalingClient->sendMessage(QJsonDocument(json).toJson(QJsonDocument::Compact));
        }
    });

    // Handle incoming local description (SDP Offer/Answer)
    m_peerConnection->onLocalDescription([this](rtc::Description description) {
        std::cout << "Local SDP Description generated." << std::endl;
        if (m_signalingClient) {
            QJsonObject json;
            json["type"] = QString::fromStdString(description.typeString());
            json["sdp"] = QString::fromStdString(std::string(description));
            // m_signalingClient->sendMessage(QJsonDocument(json).toJson(QJsonDocument::Compact));
        }
    });

    // Create a data channel
    m_dataChannel = m_peerConnection->createDataChannel("drone_data");
    m_dataChannel->onOpen([this]() {
        std::cout << "Data channel opened" << std::endl;
        m_dataChannel->send("Hello from DroneTracking WebRTC!");
    });

    m_dataChannel->onMessage([this](std::variant<rtc::binary, rtc::string> message) {
        if (std::holds_alternative<rtc::string>(message)) {
            std::string msg = std::get<rtc::string>(message);
            std::cout << "Received msg on data channel: " << msg << std::endl;
            if (m_onMessageCallback) {
                m_onMessageCallback(msg);
            }
        }
    });
}

void WebRtcManager::setOnMessageCallback(std::function<void(const std::string&)> callback) {
    m_onMessageCallback = std::move(callback);
}

void WebRtcManager::sendMessage(const std::string& message) {
    if (m_dataChannel && m_dataChannel->isOpen()) {
        m_dataChannel->send(message);
    }
}
