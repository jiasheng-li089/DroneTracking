#pragma once

#include <QWebSocket>
#include <QObject>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <thread>

#include "Signling.h"

class WebSocketSignaling : public QObject, public Signaling {
    Q_OBJECT
public:
    WebSocketSignaling(std::string websocket_url, std::string subprotocol, QObject *parent = nullptr);

    ~WebSocketSignaling() override;

    std::string exchange_offer(const std::string& offer) override;

    void end() override;

private:
    std::unique_ptr<QWebSocket> m_websocket = nullptr;

    std::string m_websocket_url;

    std::string m_subprotocol;

    long m_session_id = 0;

    long m_handle_id = 0;

    std::map<std::string, std::promise<std::string>> m_pending_offers;
    std::mutex m_pending_offers_mutex;

    // Keep-alive
    std::thread m_keepalive_thread;
    std::atomic<bool> m_keepalive_running{false};
    static constexpr std::chrono::seconds KEEPALIVE_INTERVAL{5};

    bool initialize_websocket();

    bool make_sure_session_and_handle();

    void release_websocket();

    void start_keepalive();
    void stop_keepalive();

    private slots:
        void on_connected();
        void on_text_message_received(const QString &message);
        void on_disconnected();
        void on_state_changed(QAbstractSocket::SocketState state);
        void on_error(QAbstractSocket::SocketError error);
};
