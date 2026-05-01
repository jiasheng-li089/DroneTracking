// Camera stereo calibration tool.
//
// Reads a capture session XML produced by PhotoCaptureTask (two cameras, paired
// infrared images) and runs OpenCV stereoCalibrate.  Results are written to an
// OpenCV FileStorage YAML file that downstream code (e.g. stereo rectification)
// can load directly.
//
// Usage:
//   DroneTracking_CameraCalibration \
//       --config  <session_info.xml>  \
//       --output  <calibration.yaml>  \
//       [--pattern-cols <N>]          \   # inner corners along width  (default 9)
//       [--pattern-rows <N>]          \   # inner corners along height (default 6)
//       [--square-size   <mm>]            # physical square side (default 25)

#include <QCoreApplication>
#include <QFile>
#include <QString>
#include <QXmlStreamReader>
#include <iostream>
#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// XML parsing
// ---------------------------------------------------------------------------

// Returns a map of  serial -> [photo_path, ...]  from the session XML.
static std::map<std::string, std::vector<std::string>> ParseConfig(const std::string& config_path) {
    std::map<std::string, std::vector<std::string>> result;

    QFile file(QString::fromStdString(config_path));
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        throw std::runtime_error("Cannot open config file: " + config_path);
    }

    QXmlStreamReader xml(&file);
    std::string current_serial;

    while (!xml.atEnd() && !xml.hasError()) {
        auto token = xml.readNext();
        if (token == QXmlStreamReader::StartElement) {
            if (xml.name() == QLatin1String("Camera")) {
                current_serial = xml.attributes().value("serial").toString().toStdString();
            } else if (xml.name() == QLatin1String("Photo") && !current_serial.empty()) {
                std::string path = xml.attributes().value("path").toString().toStdString();
                result[current_serial].push_back(path);
            }
        } else if (token == QXmlStreamReader::EndElement) {
            if (xml.name() == QLatin1String("Camera")) {
                current_serial.clear();
            }
        }
    }

    if (xml.hasError()) {
        throw std::runtime_error("XML parse error: " + xml.errorString().toStdString());
    }

    return result;
}

// ---------------------------------------------------------------------------
// Argument parsing
// ---------------------------------------------------------------------------

struct Args {
    std::string config_path;
    std::string output_path;
    int pattern_cols = 9;       // inner corners along X
    int pattern_rows = 6;       // inner corners along Y
    double square_size = 25.0;  // mm
};

static void PrintUsage(const char* prog) {
    std::cerr << "Usage: " << prog << " --config <xml> --output <yaml>"
              << " [--pattern-cols N] [--pattern-rows N]"
              << " [--square-size mm]\n";
}

static Args ParseArgs(int argc, char* argv[]) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("Missing value for " + arg);
            return argv[++i];
        };
        if (arg == "--config")
            a.config_path = next();
        else if (arg == "--output")
            a.output_path = next();
        else if (arg == "--pattern-cols")
            a.pattern_cols = std::stoi(next());
        else if (arg == "--pattern-rows")
            a.pattern_rows = std::stoi(next());
        else if (arg == "--square-size")
            a.square_size = std::stod(next());
        else {
            std::cerr << "Unknown argument: " << arg << "\n";
        }
    }
    return a;
}

// ---------------------------------------------------------------------------
// Corner detection for one image pair
// ---------------------------------------------------------------------------

static bool FindCorners(const cv::Mat& img, cv::Size pattern, std::vector<cv::Point2f>& corners) {
    bool found = cv::findChessboardCorners(
        img, pattern, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

    if (found) {
        cv::cornerSubPix(img, corners, {11, 11}, {-1, -1},
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    }
    return found;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char* argv[]) {
    QCoreApplication app(argc, argv);

    Args args;
    try {
        args = ParseArgs(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "Argument error: " << e.what() << "\n";
        PrintUsage(argv[0]);
        return 1;
    }

    if (args.config_path.empty() || args.output_path.empty()) {
        PrintUsage(argv[0]);
        return 1;
    }

    // ------------------------------------------------------------------
    // Load photo paths from XML
    // ------------------------------------------------------------------
    std::map<std::string, std::vector<std::string>> camera_photos;
    try {
        camera_photos = ParseConfig(args.config_path);
    } catch (const std::exception& e) {
        std::cerr << "Config error: " << e.what() << "\n";
        return 1;
    }

    if (camera_photos.size() != 2) {
        std::cerr << "Expected exactly 2 cameras in config, found " << camera_photos.size() << "\n";
        return 1;
    }

    auto it = camera_photos.begin();
    const std::string serial1 = it->first;
    const std::vector<std::string> paths1 = it->second;
    ++it;
    const std::string serial2 = it->first;
    const std::vector<std::string> paths2 = it->second;

    std::cout << "Camera 1  serial=" << serial1 << "  images=" << paths1.size() << "\n";
    std::cout << "Camera 2  serial=" << serial2 << "  images=" << paths2.size() << "\n";
    std::cout << "Chessboard pattern: " << args.pattern_cols << "x" << args.pattern_rows
              << "  square=" << args.square_size << " mm\n\n";

    // ------------------------------------------------------------------
    // Build object-point template (the same for every view)
    // ------------------------------------------------------------------
    cv::Size pattern_size(args.pattern_cols, args.pattern_rows);

    std::vector<cv::Point3f> obj_template;
    obj_template.reserve(args.pattern_cols * args.pattern_rows);
    for (int r = 0; r < args.pattern_rows; ++r) {
        for (int c = 0; c < args.pattern_cols; ++c) {
            obj_template.emplace_back(static_cast<float>(c * args.square_size),
                                      static_cast<float>(r * args.square_size), 0.0f);
        }
    }

    // ------------------------------------------------------------------
    // Detect corners in every paired image
    // ------------------------------------------------------------------
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points1, image_points2;
    cv::Size image_size;

    const size_t n_pairs = std::min(paths1.size(), paths2.size());
    int valid_pairs = 0;

    for (size_t i = 0; i < n_pairs; ++i) {
        cv::Mat img1 = cv::imread(paths1[i], cv::IMREAD_GRAYSCALE);
        cv::Mat img2 = cv::imread(paths2[i], cv::IMREAD_GRAYSCALE);

        if (img1.empty() || img2.empty()) {
            std::cout << "  pair " << i << ": cannot load images, skipping\n";
            continue;
        }
        if (image_size.empty()) {
            image_size = img1.size();
        }

        std::vector<cv::Point2f> corners1, corners2;
        bool ok1 = FindCorners(img1, pattern_size, corners1);
        bool ok2 = FindCorners(img2, pattern_size, corners2);

        if (!ok1 || !ok2) {
            std::cout << "  pair " << i << ": pattern not found"
                      << " (cam1=" << ok1 << " cam2=" << ok2 << "), skipping\n";
            continue;
        }

        object_points.push_back(obj_template);
        image_points1.push_back(corners1);
        image_points2.push_back(corners2);
        ++valid_pairs;
        std::cout << "  pair " << i << ": ok\n";
    }

    std::cout << "\nValid pairs used for calibration: " << valid_pairs << "\n";

    if (valid_pairs < 5) {
        std::cerr << "Need at least 5 valid pairs — found only " << valid_pairs << ".\n"
                  << "Check that the chessboard is visible in both cameras and that\n"
                  << "--pattern-cols / --pattern-rows match your physical target.\n";
        return 1;
    }

    // ------------------------------------------------------------------
    // Individual camera calibration (provides good initial values)
    // ------------------------------------------------------------------
    cv::Mat K1 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat K2 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D1, D2;

    double rms1 = cv::calibrateCamera(object_points, image_points1, image_size, K1, D1, cv::noArray(), cv::noArray(),
                                      cv::CALIB_FIX_ASPECT_RATIO);

    double rms2 = cv::calibrateCamera(object_points, image_points2, image_size, K2, D2, cv::noArray(), cv::noArray(),
                                      cv::CALIB_FIX_ASPECT_RATIO);

    std::cout << "\nIndividual calibration RMS  cam1=" << rms1 << "  cam2=" << rms2 << "\n";

    // ------------------------------------------------------------------
    // Stereo calibration
    // ------------------------------------------------------------------
    cv::Mat R, T, E, F;

    double stereo_rms =
        cv::stereoCalibrate(object_points, image_points1, image_points2, K1, D1, K2, D2, image_size, R, T, E, F,
                            cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_ZERO_TANGENT_DIST,
                            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5));

    std::cout << "Stereo calibration RMS reprojection error: " << stereo_rms << "\n";

    // Warn if reprojection error looks unusually high
    if (stereo_rms > 1.0) {
        std::cerr << "Warning: RMS > 1.0 px — results may be inaccurate.\n"
                  << "Ensure the chessboard pattern dimensions match exactly and that\n"
                  << "images cover the full sensor field of view.\n";
    }

    // ------------------------------------------------------------------
    // Save results
    // ------------------------------------------------------------------
    cv::FileStorage fs(args.output_path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "Cannot open output file: " << args.output_path << "\n";
        return 1;
    }

    fs << "camera1_serial" << serial1;
    fs << "camera2_serial" << serial2;
    fs << "image_width" << image_size.width;
    fs << "image_height" << image_size.height;
    fs << "pattern_cols" << args.pattern_cols;
    fs << "pattern_rows" << args.pattern_rows;
    fs << "square_size_mm" << args.square_size;
    fs << "valid_pairs" << valid_pairs;
    fs << "rms1" << rms1;
    fs << "rms2" << rms2;
    fs << "stereo_rms" << stereo_rms;

    fs << "cameras" << "{";

    fs << serial1 << "{";
    fs << "K" << K1;
    fs << "D" << D1;
    fs << "T" << T;
    fs << "R" << R;
    fs << "}";

    fs << serial2 << "{";
    fs << "K" << K2;
    fs << "D" << D2;
    fs << "T" << -R.t() * T;  // cam1 relative to cam2 is inverse of cam2 relative to cam1
    fs << "R" << R.t();       // cam1 relative to cam2 is inverse of cam2 relative to cam1
    fs << "}";

    fs << "}";

    fs << "E" << E;  // essential matrix
    fs << "F" << F;  // fundamental matrix

    fs.release();

    std::cout << "Calibration results saved to: " << args.output_path << "\n";
    return 0;
}
