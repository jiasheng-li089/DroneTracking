#ifndef DRONETRACKING_PHOTOCAPTURETASK_H
#define DRONETRACKING_PHOTOCAPTURETASK_H

#include <QImage>
#include <QObject>
#include <map>
#include <set>
#include <string>

namespace media {

class PhotoCaptureTask : public QObject {
    Q_OBJECT
public:
    PhotoCaptureTask(const std::string target_dir, QObject* parent = nullptr);
    ~PhotoCaptureTask() override;

    void on_frame(const std::string& serial, const QImage& ir_image);

    void capture_frames(const std::set<std::string>& serials);

    void finalize();

signals:
    void finalize_complete(bool success, const QString& message);

    void update_capture_status(bool capturing);

private:
    bool process_and_save();

    const std::string m_target_dir;
    std::map<std::string, std::vector<std::string>> m_captured_photo_paths;

    std::set<std::string> m_captured_serials;
    std::string m_start_capture_time;
};

} // namespace media

#endif // DRONETRACKING_PHOTOCAPTURETASK_H
