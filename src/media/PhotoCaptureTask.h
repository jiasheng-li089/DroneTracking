#ifndef DRONETRACKING_PHOTOCAPTURETASK_H
#define DRONETRACKING_PHOTOCAPTURETASK_H

#include <QImage>
#include <QObject>
#include <map>
#include <string>

namespace media {

struct CameraCapture {
    QImage ir1;
    QImage ir2;
};

class PhotoCaptureTask : public QObject {
    Q_OBJECT
public:
    PhotoCaptureTask(int camera_count, const std::string& target_dir, QObject* parent = nullptr);
    ~PhotoCaptureTask() override;

    void capture_frames(const std::string& serial, const QImage& ir1, const QImage& ir2);

signals:
    void captureComplete(bool success, const QString& message);

private:
    void process_and_save();

    int m_cameraCount = 0;
    std::string m_targetDir;
    std::map<std::string, CameraCapture> m_capturedFrames;
};

} // namespace media

#endif // DRONETRACKING_PHOTOCAPTURETASK_H
