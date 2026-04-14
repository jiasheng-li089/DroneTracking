#include "PhotoCaptureTask.h"
#include <QDir>
#include <QXmlStreamWriter>
#include <QFile>
#include <QDebug>

namespace media {
PhotoCaptureTask::PhotoCaptureTask(int camera_count, const std::string& target_dir, QObject* parent)
    : QObject(parent), m_cameraCount(camera_count), m_targetDir(target_dir) {}

PhotoCaptureTask::~PhotoCaptureTask() = default;

void PhotoCaptureTask::capture_frames(const std::string& serial, const QImage& ir1, const QImage& ir2) {
    if (m_capturedFrames.find(serial) != m_capturedFrames.end()) {
        return; // Already captured for this camera
    }

    m_capturedFrames[serial] = {ir1, ir2};

    if (m_capturedFrames.size() >= static_cast<size_t>(m_cameraCount)) {
        process_and_save();
    }
}

void PhotoCaptureTask::process_and_save() {
    QDir dir(QString::fromStdString(m_targetDir));
    if (!dir.exists()) {
        if (!dir.mkpath(".")) {
            emit captureComplete(false, "Failed to create target directory: " + QString::fromStdString(m_targetDir));
            return;
        }
    }

    QString xmlFilePath = dir.filePath("info.xml");
    QFile xmlFile(xmlFilePath);
    if (!xmlFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        emit captureComplete(false, "Failed to open XML file for writing");
        return;
    }

    QXmlStreamWriter xmlWriter(&xmlFile);
    xmlWriter.setAutoFormatting(true);
    xmlWriter.writeStartDocument();
    xmlWriter.writeStartElement("Captures");

    for (const auto& [serial, frames] : m_capturedFrames) {
        xmlWriter.writeStartElement("Camera");
        xmlWriter.writeAttribute("serial", QString::fromStdString(serial));
        
        QString ir1Name = QString::fromStdString(serial) + "_ir1.png";
        QString ir2Name = QString::fromStdString(serial) + "_ir2.png";
        QString ir1Path = dir.filePath(ir1Name);
        QString ir2Path = dir.filePath(ir2Name);

        if (!frames.ir1.isNull()) {
            frames.ir1.save(ir1Path);
            xmlWriter.writeEmptyElement("Photo");
            xmlWriter.writeAttribute("type", "infrared_1");
            xmlWriter.writeAttribute("path", ir1Name);
        }

        if (!frames.ir2.isNull()) {
            frames.ir2.save(ir2Path);
            xmlWriter.writeEmptyElement("Photo");
            xmlWriter.writeAttribute("type", "infrared_2");
            xmlWriter.writeAttribute("path", ir2Name);
        }

        xmlWriter.writeEndElement(); // Camera
    }

    xmlWriter.writeEndElement(); // Captures
    xmlWriter.writeEndDocument();
    xmlFile.close();

    emit captureComplete(true, "Successfully saved photos and info.xml to " + QString::fromStdString(m_targetDir));
}

} // namespace media
