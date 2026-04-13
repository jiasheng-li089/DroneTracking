#include "MainWindow.h"
#include "../network/WebSocketClient.h"
#include "../network/WebRtcManager.h"
#include <QWidget>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      m_webSocketClient(std::make_unique<WebSocketClient>(this)),
      m_webRtcManager(std::make_unique<WebRtcManager>())
{
    setupUi();

    connect(m_webSocketClient.get(), &WebSocketClient::connected, this, [this]() {
        appendLog("Signaling WebSocket connected!");
    });
    connect(m_webSocketClient.get(), &WebSocketClient::disconnected, this, [this]() {
        appendLog("Signaling WebSocket disconnected.");
    });
    connect(m_webSocketClient.get(), &WebSocketClient::messageReceived, this, [this](const QString& msg) {
        appendLog("Signaling message: " + msg);
        // Normally, you would parse the signaling message (e.g. SDP offer/answer or ICE candidate)
        // and pass it to m_webRtcManager.
    });
    connect(m_webSocketClient.get(), &WebSocketClient::errorOccurred, this, [this](const QString& err) {
        appendLog("Signaling Error: " + err);
    });
}

MainWindow::~MainWindow() = default;

void MainWindow::setupUi() {
    this->setWindowTitle("Drone Tracking Control");
    this->resize(800, 600);

    QWidget* centralWidget = new QWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(centralWidget);

    m_btnConnectSignaling = new QPushButton("Connect to Signaling Server", this);
    m_btnStartWebRtc = new QPushButton("Start WebRTC Connection", this);
    m_logTextEdit = new QTextEdit(this);
    m_logTextEdit->setReadOnly(true);

    layout->addWidget(m_btnConnectSignaling);
    layout->addWidget(m_btnStartWebRtc);
    layout->addWidget(m_logTextEdit);

    this->setCentralWidget(centralWidget);

    connect(m_btnConnectSignaling, &QPushButton::clicked, this, &MainWindow::onConnectSignalingClicked);
    connect(m_btnStartWebRtc, &QPushButton::clicked, this, &MainWindow::onStartWebRtcClicked);
}

void MainWindow::onConnectSignalingClicked() {
    appendLog("Connecting to signaling server...");
    // Replace with your actual signaling server WebSocket URL
    m_webSocketClient->connectToServer(QUrl("ws://localhost:8080/ws"));
}

void MainWindow::onStartWebRtcClicked() {
    appendLog("Initializing WebRTC connection...");
    m_webRtcManager->initialize(m_webSocketClient.get());
    
    m_webRtcManager->setOnMessageCallback([this](const std::string& msg) {
        QMetaObject::invokeMethod(this, [this, msg]() {
            appendLog(QString("WebRTC Data Channel: %1").arg(QString::fromStdString(msg)));
        });
    });
}

void MainWindow::appendLog(const QString& message) {
    m_logTextEdit->append(message);
}
