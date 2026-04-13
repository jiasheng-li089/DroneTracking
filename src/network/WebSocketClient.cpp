#include "network/WebSocketClient.h"

WebSocketClient::WebSocketClient(QObject *parent)
    : QObject(parent)
{
    connect(&m_webSocket, &QWebSocket::connected, this, &WebSocketClient::onConnected);
    connect(&m_webSocket, &QWebSocket::disconnected, this, &WebSocketClient::onDisconnected);
    connect(&m_webSocket, &QWebSocket::textMessageReceived, this, &WebSocketClient::onTextMessageReceived);
    connect(&m_webSocket, &QWebSocket::errorOccurred, this, &WebSocketClient::onError);
}

WebSocketClient::~WebSocketClient() {
    m_webSocket.close();
}

void WebSocketClient::connectToServer(const QUrl &url) {
    m_webSocket.open(url);
}

void WebSocketClient::disconnectFromServer() {
    m_webSocket.close();
}

void WebSocketClient::sendMessage(const QString &message) {
    if (m_webSocket.isValid()) {
        m_webSocket.sendTextMessage(message);
    }
}

void WebSocketClient::onConnected() {
    emit connected();
}

void WebSocketClient::onDisconnected() {
    emit disconnected();
}

void WebSocketClient::onTextMessageReceived(const QString &message) {
    emit messageReceived(message);
}

void WebSocketClient::onError(QAbstractSocket::SocketError error) {
    Q_UNUSED(error);
    emit errorOccurred(m_webSocket.errorString());
}
