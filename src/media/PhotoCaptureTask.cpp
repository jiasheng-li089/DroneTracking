#include "PhotoCaptureTask.h"

#include <QDebug>
#include <QDir>
#include <QFile>
#include <QXmlStreamWriter>

namespace media {

bool make_sure_dir_exists(const QString& dirPath) {
    QDir dir(dirPath);
    if (!dir.exists()) {
        return dir.mkpath(".");
    }
    return true;
}

PhotoCaptureTask::PhotoCaptureTask(const std::string target_dir, QObject* parent)
    : QObject(parent), m_target_dir(target_dir) {}

PhotoCaptureTask::~PhotoCaptureTask() = default;

void PhotoCaptureTask::capture_frames(const std::set<std::string>& serials) {
    m_captured_serials.insert(serials.begin(), serials.end());
    m_start_capture_time = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss").toStdString();
    emit update_capture_status(true);
}

void PhotoCaptureTask::finalize() { 
    if(process_and_save()) {
        m_captured_photo_paths.clear();
        m_captured_serials.clear();
        m_start_capture_time = "";
    } 
}

void PhotoCaptureTask::on_frame(const std::string& serial, const QImage& ir_image) {
    if (m_captured_serials.find(serial) != m_captured_serials.end()) {
        if(!make_sure_dir_exists(QString::fromStdString(m_target_dir))) {
            qDebug() << "Failed to create target directory:" << QString::fromStdString(m_target_dir);
            return;
        }
        
        QString fileName = QString::fromStdString(serial) + "_" + QString::fromStdString(m_start_capture_time) + "_" +
                           QString::number(m_captured_photo_paths[serial].size()) + ".png";
        auto filePath = QDir(QString::fromStdString(m_target_dir)).filePath(fileName);

        if (ir_image.save(filePath)) {
            m_captured_photo_paths[serial].push_back(filePath.toStdString());
            qDebug() << "Captured photo for serial:" << QString::fromStdString(serial) << "saved to:" << filePath;
        } else {
            qDebug() << "Failed to save photo for serial:" << QString::fromStdString(serial);
        }

        m_captured_serials.erase(serial);
    }

    if (m_captured_serials.size() == 0) {
        emit update_capture_status(false);
    }
}

bool PhotoCaptureTask::process_and_save() {
    if (!make_sure_dir_exists(QString::fromStdString(m_target_dir))) {
        emit finalize_complete(false, "Failed to create target directory: " + QString::fromStdString(m_target_dir));
        return false;
    }

    auto fiel_name = QString::fromStdString(m_start_capture_time) + "_info.xml";
    QString xmlFilePath = QDir(QString::fromStdString(m_target_dir)).filePath(fiel_name);
    QFile xmlFile(xmlFilePath);
    if (!xmlFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        emit finalize_complete(false, "Failed to open XML file for writing");
        return false;
    }

    QXmlStreamWriter xml_writer(&xmlFile);
    xml_writer.setAutoFormatting(true);
    xml_writer.writeStartDocument();
    xml_writer.writeStartElement("Captures");

    for (const auto& [serial, photo_paths]: m_captured_photo_paths) {
        xml_writer.writeStartElement("Camera");
        xml_writer.writeAttribute("serial", QString::fromStdString(serial));

        for (const auto& path : photo_paths) {
            xml_writer.writeEmptyElement("Photo");
            xml_writer.writeAttribute("path", QString::fromStdString(path));
        }

        xml_writer.writeEndElement(); // Camera
    }
    xml_writer.writeEndElement();  // Captures
    xml_writer.writeEndDocument();
    xmlFile.close();

    emit finalize_complete(true, "Successfully saved photos and info.xml to " + QString::fromStdString(m_target_dir));
    return true;
}

}  // namespace media
