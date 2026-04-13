#include "MainWindow.h"
#include "../network/WebSocketClient.h"
#include "../network/WebRtcManager.h"
#include <QWidget>
#include "CameraWidget.h"
#include "../camera/RealSenseManager.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      m_webSocketClient(std::make_unique<WebSocketClient>(this)),
      m_webRtcManager(std::make_unique<WebRtcManager>()),
      m_rsManager(std::make_unique<RealSenseManager>())
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

    connect(m_rsManager.get(), &RealSenseManager::frameReceived, this, &MainWindow::onFrameReceived);
    connect(m_rsManager.get(), &RealSenseManager::errorOccurred, this, &MainWindow::onCameraError);

    m_rsManager->startCameras();
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
    m_logTextEdit->setMaximumHeight(150);

    m_camerasContainer = new QWidget(this);
    m_camerasLayout = new QGridLayout(m_camerasContainer);

    layout->addWidget(m_btnConnectSignaling);
    layout->addWidget(m_btnStartWebRtc);
    layout->addWidget(m_camerasContainer);
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

void MainWindow::onFrameReceived(int cameraId, const QImage& img) {
    if (!m_cameraWidgets.contains(cameraId)) {
        CameraWidget* widget = new CameraWidget(this);
        widget->setMinimumSize(320, 240);
        
        int count = m_cameraWidgets.size();
        int row = count / 2;
        int col = count % 2;
        m_camerasLayout->addWidget(widget, row, col);
        
        m_cameraWidgets.insert(cameraId, widget);
        appendLog(QString("Detected new camera stream: ID %1").arg(cameraId));
    }
    
    m_cameraWidgets[cameraId]->updateFrame(img);
}

void MainWindow::onCameraError(const QString& err) {
    appendLog(err);
}
