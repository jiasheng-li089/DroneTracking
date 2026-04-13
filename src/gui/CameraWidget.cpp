#include "CameraWidget.h"
#include <QPainter>

CameraWidget::CameraWidget(QWidget* parent) : QOpenGLWidget(parent) {
    // Initialization, etc.
}

CameraWidget::~CameraWidget() = default;

void CameraWidget::updateFrame(const QImage& frame) {
    m_currentFrame = frame;
    update(); // Schedule a repaint
}

void CameraWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    
    QPainter painter(this);
    painter.fillRect(rect(), Qt::black); // Clear background to black
    
    if (!m_currentFrame.isNull()) {
        // Scaled to fit widget maintaining aspect ratio
        QImage scaled = m_currentFrame.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
        int x = (width() - scaled.width()) / 2;
        int y = (height() - scaled.height()) / 2;
        painter.drawImage(x, y, scaled);
    }
}
