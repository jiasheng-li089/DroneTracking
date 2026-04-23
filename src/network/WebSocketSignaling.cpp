#include "WebSocketSignaling.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QNetworkRequest>
#include <chrono>

#include "../logger.h"

#define SUCCESS "success"

#define FIELD_ID "id"
#define FIELD_JANUS "janus"
#define FIELD_SESSION_ID "session_id"
#define FIELD_HANDLE_ID "handle_id"
#define FIELD_PLUGIN "plugin"
#define FIELD_DATA "data"
#define FIELD_TRANSACTION "transaction"
#define FIELD_ERROR "error"
#define FIELD_REASON "reason"
#define FIELD_CODE "code"

const static int VIDEO_ROOM = 1234;
const static std::string VIDEO_ROOM_PIN = "adminpwd";

struct JanusReq {
    /* data */
    std::string janus;
    std::string transaction = "txn_" + std::to_string(std::rand());

    long session_id = 0;
    long handle_id = 0;
    std::string plugin;

    QJsonObject toJsonObject() const {
        QJsonObject obj;
        obj[FIELD_JANUS] = QString::fromStdString(janus);
        obj[FIELD_TRANSACTION] = QString::fromStdString(transaction);
        if (session_id != 0) {
            obj[FIELD_SESSION_ID] = static_cast<qint64>(session_id);
        }
        if (handle_id != 0) {
            obj[FIELD_HANDLE_ID] = static_cast<qint64>(handle_id);
        }
        if (!plugin.empty()) {
            obj[FIELD_PLUGIN] = QString::fromStdString(plugin);
        }
        return obj;
    }
};

struct Jsep {
    std::string type = "";
    std::string sdp;

    QJsonObject toJsonObject() const {
        QJsonObject obj;
        obj["type"] = QString::fromStdString(type);
        obj["sdp"] = QString::fromStdString(sdp);
        return obj;
    }

    static Jsep fromJson(const QJsonObject& obj) {
        Jsep jsep;
        jsep.type = obj["type"].toString().toStdString();
        jsep.sdp = obj["sdp"].toString().toStdString();
        return jsep;
    }
};

struct Error {
    int code = 0;
    std::string reason = "";

    static Error fromJson(const QJsonObject& obj) {
        Error error;
        error.code = obj[FIELD_CODE].toInt();
        error.reason = obj[FIELD_REASON].toString().toStdString();
        return error;
    }
};

struct PluginData {
    std::string plugin = "";

    QJsonObject data;

    static PluginData fromJson(const QJsonObject& obj) {
        PluginData pluginData;
        pluginData.plugin = obj[FIELD_PLUGIN].toString().toStdString();
        pluginData.data = obj[FIELD_DATA].toObject();
        return pluginData;
    }
};

struct JoinAndConfigureBody {
    std::string request = "joinandconfigure";
    int room = VIDEO_ROOM;
    std::string ptype = "publisher";
    std::string display = "drone_position_tracker";

    QJsonObject toJsonObject() const {
        QJsonObject obj;
        obj["request"] = QString::fromStdString(request);
        obj["room"] = static_cast<qint64>(room);
        obj["pin"] = QString::fromStdString(VIDEO_ROOM_PIN);
        obj["ptype"] = QString::fromStdString(ptype);
        obj["display"] = QString::fromStdString(display);
        return obj;
    }
};

struct JoinAndConfigureReq {
    /* data */
    std::string janus = "message";
    std::string transaction = "txn_" + std::to_string(std::rand());

    long session_id = 0;
    long handle_id = 0;

    struct Jsep jsep;
    struct JoinAndConfigureBody body;

    QJsonObject toJsonObject() const {
        QJsonObject obj;
        obj[FIELD_JANUS] = QString::fromStdString(janus);
        obj[FIELD_TRANSACTION] = QString::fromStdString(transaction);
        if (session_id != 0) {
            obj[FIELD_SESSION_ID] = static_cast<qint64>(session_id);
        }
        if (handle_id != 0) {
            obj[FIELD_HANDLE_ID] = static_cast<qint64>(handle_id);
        }
        obj["body"] = body.toJsonObject();
        obj["jsep"] = jsep.toJsonObject();
        return obj;
    }
};

struct CreateSessionAndHandleResp {
    std::string janus;
    std::string transaction;
    long id;
    std::string error;

    QJsonObject data;

    static CreateSessionAndHandleResp fromJson(const QJsonObject& obj) {
        CreateSessionAndHandleResp resp;
        resp.janus = obj[FIELD_JANUS].toString().toStdString();
        resp.transaction = obj[FIELD_TRANSACTION].toString().toStdString();

        if (SUCCESS == resp.janus) {
            resp.id = obj[FIELD_DATA].toObject()[FIELD_ID].toVariant().toLongLong();
        } else {
            resp.id = 0;
            resp.error =
                obj.contains("error") ? obj[FIELD_ERROR].toObject()[FIELD_REASON].toString().toStdString() : "";
        }
        resp.data = obj[FIELD_DATA].toObject();
        return resp;
    }
};

struct PluginResp {
    std::string janus;
    std::string transaction;

    struct Jsep jsep;
    struct Error error;
    struct PluginData plugin_data;

    static PluginResp fromJson(const QJsonObject& obj) {
        PluginResp resp;
        resp.janus = obj[FIELD_JANUS].toString().toStdString();
        resp.transaction = obj[FIELD_TRANSACTION].toString().toStdString();

        resp.error = obj.contains(FIELD_ERROR) ? Error::fromJson(obj[FIELD_ERROR].toObject()) : Error();

        resp.jsep = obj.contains("jsep") ? Jsep::fromJson(obj["jsep"].toObject()) : Jsep();

        resp.plugin_data = obj.contains(FIELD_DATA) ? PluginData::fromJson(obj) : PluginData();

        return resp;
    }
};

WebSocketSignaling::WebSocketSignaling(std::string websocket_url, std::string subprotocol, QObject* parent)
    : QObject(parent), m_websocket_url(websocket_url), m_subprotocol(subprotocol) {
    m_event_loop_thread = std::thread([this]() {
        // All Qt objects must be created on the thread that runs the event loop
        QObject thread_context;
        QEventLoop event_loop;
        QTimer keepalive_timer;

        keepalive_timer.setInterval(20 * 1000);
        QObject::connect(&keepalive_timer, &QTimer::timeout, [this]() { keep_alive_event(); });
        keepalive_timer.start();

        {
            std::lock_guard<std::mutex> lock(m_thread_ready_mutex);
            m_thread_context = &thread_context;
            m_event_loop = &event_loop;
            m_keepalive_timer = &keepalive_timer;
            m_thread_ready = true;
        }
        m_thread_ready_cv.notify_one();

        spdlog::debug("WebSocket event loop thread started");
        event_loop.exec();
        spdlog::debug("WebSocket event loop thread stopped");

        m_thread_context = nullptr;
        m_event_loop = nullptr;
        m_keepalive_timer = nullptr;
    });

    // Wait until the thread has created its Qt objects
    std::unique_lock<std::mutex> lock(m_thread_ready_mutex);
    m_thread_ready_cv.wait(lock, [this]() { return m_thread_ready; });
}

WebSocketSignaling::~WebSocketSignaling() {
    m_pending_offers.clear();
    release_websocket();

    if (m_event_loop) {
        m_event_loop->quit();
    }
    if (m_event_loop_thread.joinable()) {
        m_event_loop_thread.join();
    }
}

void WebSocketSignaling::end() {
    if (0 != m_handle_id && 0 != m_session_id) {
        struct JanusReq leave_req;
        leave_req.janus = "leave";
        leave_req.session_id = m_session_id;
        leave_req.handle_id = m_handle_id;
        send_text_message(QJsonDocument(leave_req.toJsonObject()).toJson(QJsonDocument::Compact));
    }

    if (0 != m_handle_id) {
        struct JanusReq detach_req;
        detach_req.janus = "detach";
        detach_req.session_id = m_session_id;
        detach_req.handle_id = m_handle_id;
        send_text_message(QJsonDocument(detach_req.toJsonObject()).toJson(QJsonDocument::Compact));
    }

    if (0 != m_session_id) {
        struct JanusReq destroy_req;
        destroy_req.janus = "destroy";
        destroy_req.session_id = m_session_id;
        send_text_message(QJsonDocument(destroy_req.toJsonObject()).toJson(QJsonDocument::Compact));
    }

    release_websocket();
}

std::string WebSocketSignaling::exchange_offer(const std::string& offer) {
    if (!initialize_websocket()) {
        throw std::runtime_error("Failed to initialize WebSocket connection");
    }

    if (!make_sure_session_and_handle()) {
        throw std::runtime_error("Failed to create session and handle on the server");
    }

    struct JoinAndConfigureReq req;
    req.session_id = m_session_id;
    req.handle_id = m_handle_id;
    req.jsep.type = "offer";
    req.jsep.sdp = offer;

    // Create a promise to wait for the response
    std::promise<std::string> response_promise;
    std::future<std::string> response_future = response_promise.get_future();
    {
        std::lock_guard<std::mutex> lock(m_pending_offers_mutex);
        m_pending_offers[req.transaction] = std::move(response_promise);
    }
    send_text_message(QJsonDocument(req.toJsonObject()).toJson(QJsonDocument::Compact));

    auto wait_status = response_future.wait_for(std::chrono::seconds(10));
    if (wait_status != std::future_status::ready) {
        // Timeout occurred
        spdlog::error("Timeout waiting for exchanging the offer with remote server, transaction id: {}",
                      req.transaction);
        {
            std::lock_guard<std::mutex> lock(m_pending_offers_mutex);
            m_pending_offers.erase(req.transaction);  // Clean up the pending offer
        }
        throw std::runtime_error("Timeout waiting for response to offer");
    }

    auto message = response_future.get();
    spdlog::debug("Received response for offer exchange, transaction id: {}, message: {}", req.transaction, message);
    PluginResp plugin_resp =
        PluginResp::fromJson(QJsonDocument::fromJson(QString::fromStdString(message).toUtf8()).object());

    if (0 == plugin_resp.error.code) {
        spdlog::debug("Offer exchange successful, remote answer: {}", plugin_resp.jsep.sdp);
        return plugin_resp.jsep.sdp;  // Return the SDP from the response
    } else {
        spdlog::error("Offer exchange failed, transaction id: {}, error code: {}, reason: {}", req.transaction,
                      plugin_resp.error.code, plugin_resp.error.reason);
        throw std::runtime_error("Offer exchange failed: " + plugin_resp.error.reason);
    }
}

bool WebSocketSignaling::initialize_websocket() {
    // Initialize the WebSocket connection using m_websocket_url and m_subprotocol
    // Set up necessary signal-slot connections for handling WebSocket events
    // Return true if initialization is successful, false otherwise
    if (m_websocket_url.empty()) {
        return false;
    }
    if (m_subprotocol.empty()) {
        return false;
    }

    if (nullptr != m_websocket && m_websocket->isValid() && m_websocket->state() == QAbstractSocket::ConnectedState) {
        return true;  // Already initialized
    }

    release_websocket();  // Ensure any existing connection is cleaned up

    // Register the connection promise before opening the socket so on_connected can fulfill it
    std::promise<std::string> connection_promise;
    std::future<std::string> connection_future = connection_promise.get_future();
    {
        std::lock_guard<std::mutex> lock(m_pending_offers_mutex);
        m_pending_offers["__connection__"] = std::move(connection_promise);
    }

    // QWebSocket must be created on the event loop thread so its signals are dispatched there
    QMetaObject::invokeMethod(m_thread_context, [this]() {
        m_websocket = std::make_unique<QWebSocket>();

        QNetworkRequest request(QUrl(QString::fromStdString(m_websocket_url)));
        request.setRawHeader("Sec-WebSocket-Protocol", QString::fromStdString(m_subprotocol).toUtf8());

        // DirectConnection: slots run on the event loop thread (where the socket lives)
        connect(m_websocket.get(), &QWebSocket::textMessageReceived, this,
                &WebSocketSignaling::on_text_message_received, Qt::DirectConnection);
        connect(m_websocket.get(), &QWebSocket::disconnected, this,
                &WebSocketSignaling::on_disconnected, Qt::DirectConnection);
        connect(m_websocket.get(), &QWebSocket::stateChanged, this,
                &WebSocketSignaling::on_state_changed, Qt::DirectConnection);
        connect(m_websocket.get(), &QWebSocket::errorOccurred, this,
                &WebSocketSignaling::on_error, Qt::DirectConnection);
        connect(m_websocket.get(), &QWebSocket::connected, this,
                &WebSocketSignaling::on_connected, Qt::DirectConnection);

        m_websocket->open(request);
    }, Qt::BlockingQueuedConnection);

    // Calling thread waits here; event loop thread processes TCP events and fulfills the promise
    auto wait_status = connection_future.wait_for(std::chrono::seconds(10));
    if (wait_status != std::future_status::ready) {
        spdlog::error("Timeout waiting for WebSocket connection to be established");
        std::lock_guard<std::mutex> lock(m_pending_offers_mutex);
        m_pending_offers.erase("__connection__");
    }

    if (!m_websocket || m_websocket->state() != QAbstractSocket::ConnectedState) {
        release_websocket();
        return false;
    }

    return true;
}

void WebSocketSignaling::release_websocket() {
    if (!m_websocket) return;

    m_session_id = 0;
    m_handle_id = 0;
    m_pending_offers.clear();

    // Socket lives on the event loop thread; must be destroyed there
    QMetaObject::invokeMethod(m_thread_context, [this]() {
        if (m_websocket) {
            m_websocket->abort();
            disconnect(m_websocket.get(), nullptr, nullptr, nullptr);
            m_websocket.reset();
        }
    }, Qt::BlockingQueuedConnection);
}

void WebSocketSignaling::keep_alive_event() {
    if (m_session_id == 0) return;

    struct JanusReq keepalive_req;
    keepalive_req.janus = "keepalive";
    keepalive_req.session_id = m_session_id;
    send_text_message(QJsonDocument(keepalive_req.toJsonObject()).toJson(QJsonDocument::Compact));
}

bool WebSocketSignaling::make_sure_session_and_handle() {
    // Ensure that a session and handle are created on the server side
    // This may involve sending specific messages and waiting for responses
    // Return true if successful, false otherwise
    if (m_session_id == 0) {
        // create a session first
        struct JanusReq create_session_req;
        create_session_req.janus = "create";

        QJsonObject json_msg = create_session_req.toJsonObject();

        std::promise<std::string> session_promise;
        std::future<std::string> session_response_future = session_promise.get_future();
        {
            std::lock_guard<std::mutex> lock(m_pending_offers_mutex);
            m_pending_offers[create_session_req.transaction] = std::move(session_promise);
        }
        send_text_message(QJsonDocument(json_msg).toJson(QJsonDocument::Compact));

        auto wait_status = session_response_future.wait_for(std::chrono::seconds(10));
        if (wait_status != std::future_status::ready) {
            // timeout waiting for session creation response, clean up the pending offer and return false
            spdlog::error("Timeout waiting for session creation response");

            std::lock_guard<std::mutex> lock(m_pending_offers_mutex);
            m_pending_offers.erase(create_session_req.transaction);
            return false;
        }
        std::string session_response = session_response_future.get();

        QJsonObject session_response_json =
            QJsonDocument::fromJson(QString::fromStdString(session_response).toUtf8()).object();
        CreateSessionAndHandleResp create_session_resp = CreateSessionAndHandleResp::fromJson(session_response_json);

        if (0 != create_session_resp.id) {
            m_session_id = create_session_resp.id;
            spdlog::info("Session created with ID: {}", m_session_id);
        } else {
            spdlog::error("Failed to create session: {}", create_session_resp.error);
            return false;
        }
    }

    if (0 == m_handle_id) {
        // attach this session to the video room plugin
        struct JanusReq attach_req;
        attach_req.janus = "attach";
        attach_req.session_id = m_session_id;
        attach_req.plugin = "janus.plugin.videoroom";

        QJsonObject json_msg = attach_req.toJsonObject();

        std::promise<std::string> attach_promise;
        std::future<std::string> attach_response_future = attach_promise.get_future();
        {
            std::lock_guard<std::mutex> lock(m_pending_offers_mutex);
            m_pending_offers[attach_req.transaction] = std::move(attach_promise);
        }
        send_text_message(QJsonDocument(json_msg).toJson(QJsonDocument::Compact));

        auto wait_status = attach_response_future.wait_for(std::chrono::seconds(10));
        if (wait_status != std::future_status::ready) {
            // timeout waiting for attach response, clean up the pending offer and return false
            spdlog::error("Timeout waiting for attach response");

            std::lock_guard<std::mutex> lock(m_pending_offers_mutex);
            m_pending_offers.erase(attach_req.transaction);
            return false;
        }
        std::string attach_response = attach_response_future.get();

        QJsonObject attach_response_json =
            QJsonDocument::fromJson(QString::fromStdString(attach_response).toUtf8()).object();
        CreateSessionAndHandleResp attach_resp = CreateSessionAndHandleResp::fromJson(attach_response_json);

        if (0 != attach_resp.id) {
            m_handle_id = attach_resp.id;
            spdlog::info("Handle attached with ID: {}", m_handle_id);
        } else {
            spdlog::error("Failed to attach handle: {}", attach_resp.error);
            return false;
        }
    }

    return true;  // Placeholder return value
}

void WebSocketSignaling::send_text_message(const QString& message) {
    // Socket lives on the event loop thread; dispatch there (QueuedConnection = non-blocking)
    QMetaObject::invokeMethod(m_thread_context, [this, message]() {
        if (m_websocket && m_websocket->state() == QAbstractSocket::ConnectedState) {
            auto bytes = m_websocket->sendTextMessage(message);
            m_websocket->flush();
            if (bytes <= 0) {
                spdlog::error("Failed to send text message: {}", message.toStdString());
            } else {
                spdlog::debug("Sent text message: {}", message.toStdString());
            }
        } else {
            spdlog::error("Cannot send message, WebSocket is not connected: {}", message.toStdString());
        }
    }, Qt::QueuedConnection);
}

void WebSocketSignaling::on_connected() {
    // Handle WebSocket connected event
    spdlog::info("WebSocket connected");
    {
        std::lock_guard<std::mutex> lock(m_pending_offers_mutex);
        auto it = m_pending_offers.find("__connection__");
        if (it != m_pending_offers.end()) {
            it->second.set_value("connected");
            m_pending_offers.erase(it);
        }
    }
}

void WebSocketSignaling::on_text_message_received(const QString& message) {
    // Handle incoming text message
    spdlog::debug("Text message received: {}", message.toStdString());

    QJsonObject json_msg = QJsonDocument::fromJson(message.toUtf8()).object();

    QString janus_msg_type = json_msg["janus"].toString();
    if ("ack" == janus_msg_type) {
        return;
    }

    auto transaction = json_msg["transaction"].toString().toStdString();
    if (transaction.empty()) {
        spdlog::error("Received message without transaction ID");
    } else {
        // handle the transaction
        spdlog::info("Received message with transaction ID: {}", transaction);
        std::lock_guard<std::mutex> lock(m_pending_offers_mutex);
        auto it = m_pending_offers.find(transaction);
        if (it != m_pending_offers.end()) {
            // TODO still need to check if the message is valid or not
            it->second.set_value(message.toStdString());
            m_pending_offers.erase(it);
        } else {
            spdlog::error("No pending offer found for transaction ID: {}", transaction);
        }
    }
}

void WebSocketSignaling::on_disconnected() {
    // Handle WebSocket disconnected event
    spdlog::info("WebSocket disconnected");
    {
        std::lock_guard<std::mutex> lock(m_pending_offers_mutex);
        for (auto& [transaction, promise] : m_pending_offers) {
            promise.set_exception(std::make_exception_ptr(std::runtime_error("WebSocket disconnected")));
        }
        m_pending_offers.clear();
    }
}

void WebSocketSignaling::on_state_changed(QAbstractSocket::SocketState state) {
    static constexpr std::array<std::string_view, 7> stateNames = {
        "UnconnectedState", "HostLookupState", "ConnectingState", "ConnectedState",
        "BoundState",       "ClosingState",    "ListeningState"};
    auto idx = static_cast<int>(state);
    std::string_view name = (idx >= 0 && idx < static_cast<int>(stateNames.size())) ? stateNames[idx] : "UnknownState";
    spdlog::info("WebSocket state changed({}): {}", static_cast<int>(state), name);
}

void WebSocketSignaling::on_error(QAbstractSocket::SocketError error) {
    // Handle WebSocket error
    spdlog::error("WebSocket error occurred: {} ({})", m_websocket->errorString().toStdString(),
                  static_cast<int>(error));
}