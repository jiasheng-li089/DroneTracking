#pragma once
#include <QOpenGLWidget>
#include <QImage>

class CameraWidget : public QOpenGLWidget {
    Q_OBJECT
public:
    explicit CameraWidget(QWidget* parent = nullptr);
    ~CameraWidget() override;

public slots:
    void update_frame(const QImage& frame);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QImage m_current_frame;
};
