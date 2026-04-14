#include "PhotoCaptureWindow.h"
#include <QWidget>
#include "CameraWidget.h"
#include "../camera/RealSenseManager.h"

PhotoCaptureWindow::PhotoCaptureWindow(QWidget *parent)
    : QMainWindow(parent),
      m_rsManager(std::make_unique<RealSenseManager>())
{
    setupUi();

    connect(m_rsManager.get(), &RealSenseManager::frameReceived, this, &PhotoCaptureWindow::onFrameReceived);
    connect(m_rsManager.get(), &RealSenseManager::errorOccurred, this, &PhotoCaptureWindow::onCameraError);

    m_rsManager->startCameras();
}

PhotoCaptureWindow::~PhotoCaptureWindow() = default;

void PhotoCaptureWindow::setupUi() {
    this->setWindowTitle("Drone Tracking Control");
    this->resize(800, 600);

    // ReSharper disable once CppDFAMemoryLeak
    const auto centralWidget = new QWidget(this);
    // ReSharper disable once CppDFAMemoryLeak
    const auto layout = new QVBoxLayout(centralWidget);

    m_btnConnectSignaling = new QPushButton("Capture", centralWidget);

    m_logTextEdit = new QTextEdit(centralWidget);
    m_logTextEdit->setMaximumHeight(150);
    m_logTextEdit->setReadOnly(true);

    m_camerasContainer = new QWidget(centralWidget);
    m_camerasLayout = new QGridLayout(m_camerasContainer);

    layout->addWidget(m_btnConnectSignaling);
    layout->addWidget(m_camerasContainer);
    layout->addWidget(m_logTextEdit);

    this->setCentralWidget(centralWidget);

    connect(m_btnConnectSignaling, &QPushButton::clicked, this, &PhotoCaptureWindow::onCapturePhotos);
}

void PhotoCaptureWindow::onCapturePhotos() const {
    appendLog("Capturing photos");
    // Replace with your actual signaling server WebSocket URL
    // TODO capture photos
}


void PhotoCaptureWindow::appendLog(const QString& message) const {
    m_logTextEdit->append(message);
}

void PhotoCaptureWindow::onFrameReceived(int cameraId, const QImage& img) {
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

void PhotoCaptureWindow::onCameraError(const QString& err) {
    appendLog(err);
}
