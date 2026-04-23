#pragma once

#include <QWebSocket>
#include <QObject>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <thread>
#include <QEventLoop>
#include <QTimer>

#include "Signling.h"


class QEventLoop;
class QTimer;

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

    // Event loop thread — all Qt objects below are owned by this thread
    std::thread m_event_loop_thread;
    QEventLoop* m_event_loop = nullptr;
    QTimer* m_keepalive_timer = nullptr;
    QObject* m_thread_context = nullptr;  // lives on event loop thread; used for invokeMethod dispatch
    std::mutex m_thread_ready_mutex;
    std::condition_variable m_thread_ready_cv;
    bool m_thread_ready = false;


    bool initialize_websocket();

    bool make_sure_session_and_handle();

    void release_websocket();

    void keep_alive_event();

    void send_text_message(const QString &message);

    private slots:
        void on_connected();
        void on_text_message_received(const QString &message);
        void on_disconnected();
        void on_state_changed(QAbstractSocket::SocketState state);
        void on_error(QAbstractSocket::SocketError error);
};
