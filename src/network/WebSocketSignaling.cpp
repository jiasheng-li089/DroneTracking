#include "WebSocketSignaling.h"

#include <QEventLoop>
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
            obj[FIELD_SESSION_ID] = QString::number(session_id);
        }
        if (handle_id != 0) {
            obj[FIELD_HANDLE_ID] = QString::number(handle_id);
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
        obj["room"] = QString::number(room);
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
            obj[FIELD_SESSION_ID] = QString::number(session_id);
        }
        if (handle_id != 0) {
            obj[FIELD_HANDLE_ID] = QString::number(handle_id);
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
    // Constructor implementation (if needed)
}

WebSocketSignaling::~WebSocketSignaling() {
    m_pending_offers.clear();
    release_websocket();
}

void WebSocketSignaling::end() {
    if (0 != m_handle_id && 0 != m_session_id) {
        struct JanusReq leave_req;
        leave_req.janus = "leave";
        leave_req.session_id = m_session_id;
        leave_req.handle_id = m_handle_id;
        m_websocket->sendTextMessage(QJsonDocument(leave_req.toJsonObject()).toJson(QJsonDocument::Compact));
    }

    if (0 != m_handle_id) {
        struct JanusReq detach_req;
        detach_req.janus = "detach";
        detach_req.session_id = m_session_id;
        detach_req.handle_id = m_handle_id;
        m_websocket->sendTextMessage(QJsonDocument(detach_req.toJsonObject()).toJson(QJsonDocument::Compact));
    }

    if (0 != m_session_id) {
        struct JanusReq destroy_req;
        destroy_req.janus = "destroy";
        destroy_req.session_id = m_session_id;
        m_websocket->sendTextMessage(QJsonDocument(destroy_req.toJsonObject()).toJson(QJsonDocument::Compact));
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
    m_websocket->sendTextMessage(QJsonDocument(req.toJsonObject()).toJson(QJsonDocument::Compact));

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
    PluginResp plugin_resp = PluginResp::fromJson(QJsonDocument::fromJson(QString::fromStdString(message).toUtf8()).object());

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

    m_websocket = std::make_unique<QWebSocket>();

    QNetworkRequest request(QUrl(QString::fromStdString(m_websocket_url)));
    request.setRawHeader("Sec-WebSocket-Protocol", QString::fromStdString(m_subprotocol).toUtf8());

    connect(m_websocket.get(), &QWebSocket::textMessageReceived, this, &WebSocketSignaling::on_text_message_received);
    connect(m_websocket.get(), &QWebSocket::disconnected, this, &WebSocketSignaling::on_disconnected);
    connect(m_websocket.get(), &QWebSocket::stateChanged, this, &WebSocketSignaling::on_state_changed);

    QEventLoop loop;
    connect(m_websocket.get(), &QWebSocket::connected, &loop, &QEventLoop::quit);
    connect(m_websocket.get(), &QWebSocket::errorOccurred, &loop, &QEventLoop::quit);

    m_websocket->open(request);

    loop.exec();  // Wait for connection or error

    if (m_websocket->state() != QAbstractSocket::ConnectedState) {
        release_websocket();
        return false;  // Connection failed
    }

    connect(m_websocket.get(), &QWebSocket::errorOccurred, this, &WebSocketSignaling::on_error);
    connect(m_websocket.get(), &QWebSocket::connected, this, &WebSocketSignaling::on_connected);

    start_keepalive();

    return true;
}

void WebSocketSignaling::release_websocket() {
    if (m_websocket) {
        stop_keepalive();
        m_session_id = 0;
        m_handle_id = 0;
        m_websocket->abort();
        disconnect(m_websocket.get(), nullptr, this, nullptr);  // Disconnect all signals
        m_pending_offers.clear();
        m_websocket.reset();
        m_websocket = nullptr;
    }
}

void WebSocketSignaling::start_keepalive() {
    m_keepalive_running = true;
    m_keepalive_thread = std::thread([this]() {
        spdlog::debug("Keep-alive thread started");
        while (m_keepalive_running) {
            std::this_thread::sleep_for(KEEPALIVE_INTERVAL);

            if (!m_keepalive_running) break;

            struct JanusReq keepalive_req;
            keepalive_req.janus = "keepalive";

            if (m_session_id != 0) {
                keepalive_req.session_id = m_session_id;
            } else {
                continue;
            }

            QString payload = QJsonDocument(keepalive_req.toJsonObject()).toJson(QJsonDocument::Compact);

            if (m_websocket == nullptr || m_websocket->state() != QAbstractSocket::ConnectedState) {
                spdlog::warn("WebSocket is not connected, skipping keep-alive");
                continue;
            }

            m_websocket->sendTextMessage(payload);

            spdlog::debug("Sent keep-alive message: {}", payload.toStdString());
        }
        spdlog::debug("Keep-alive thread stopped");
    });
}

void WebSocketSignaling::stop_keepalive() {
    m_keepalive_running = false;
    if (m_keepalive_thread.joinable()) {
        m_keepalive_thread.join();
    }
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
        m_websocket->sendTextMessage(QJsonDocument(json_msg).toJson(QJsonDocument::Compact));

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
        m_websocket->sendTextMessage(QJsonDocument(json_msg).toJson(QJsonDocument::Compact));

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

void WebSocketSignaling::on_connected() {
    // Handle WebSocket connected event
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
}

void WebSocketSignaling::on_state_changed(QAbstractSocket::SocketState state) {
    // Handle WebSocket state change
    spdlog::info("WebSocket state changed: {}", static_cast<int>(state));
}

void WebSocketSignaling::on_error(QAbstractSocket::SocketError error) {
    // Handle WebSocket error
    spdlog::error("WebSocket error occurred: {} ({})", m_websocket->errorString().toStdString(), static_cast<int>(error));
}