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
//       [--pattern-cols <N>]          \   # inner corners along width  (default
//       9)
//       [--pattern-rows <N>]          \   # inner corners along height (default
//       6)
//       [--square-size   <mm>]            # physical square side (default 25)

#include <QCoreApplication>
#include <QFile>
#include <QString>
#include <QXmlStreamReader>
#include <iostream>
#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// ---------------------------------------------------------------------------
// XML parsing
// ---------------------------------------------------------------------------

// Returns a map of  serial -> [photo_path, ...]  from the session XML.
static std::map<std::string, std::vector<std::string>>
ParseConfig(const std::string &config_path) {
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
        current_serial =
            xml.attributes().value("serial").toString().toStdString();
      } else if (xml.name() == QLatin1String("Photo") &&
                 !current_serial.empty()) {
        std::string path =
            xml.attributes().value("path").toString().toStdString();
        result[current_serial].push_back(path);
      }
    } else if (token == QXmlStreamReader::EndElement) {
      if (xml.name() == QLatin1String("Camera")) {
        current_serial.clear();
      }
    }
  }

  if (xml.hasError()) {
    throw std::runtime_error("XML parse error: " +
                             xml.errorString().toStdString());
  }

  return result;
}

// ---------------------------------------------------------------------------
// Argument parsing
// ---------------------------------------------------------------------------

struct Args {
  std::string config_path;
  std::string output_path;
  int pattern_cols = 9;      // inner corners along X
  int pattern_rows = 6;      // inner corners along Y
  double square_size = 25.0; // mm
};

static void PrintUsage(const char *prog) {
  std::cerr << "Usage: " << prog << " --config <xml> --output <yaml>"
            << " [--pattern-cols N] [--pattern-rows N]"
            << " [--square-size mm]\n";
}

static Args ParseArgs(int argc, char *argv[]) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc)
        throw std::runtime_error("Missing value for " + arg);
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

static bool FindCorners(const cv::Mat &img, cv::Size pattern,
                        std::vector<cv::Point2f> &corners) {
  bool found = cv::findChessboardCorners(img, pattern, corners,
                                         cv::CALIB_CB_ADAPTIVE_THRESH |
                                             cv::CALIB_CB_NORMALIZE_IMAGE |
                                             cv::CALIB_CB_FAST_CHECK);

  if (found) {
    cv::cornerSubPix(
        img, corners, {11, 11}, {-1, -1},
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
                         0.001));
  }
  return found;
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char *argv[]) {
  QCoreApplication app(argc, argv);

  Args args;
  try {
    args = ParseArgs(argc, argv);
  } catch (const std::exception &e) {
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
  } catch (const std::exception &e) {
    std::cerr << "Config error: " << e.what() << "\n";
    return 1;
  }

  std::vector<std::string> camera_serials;
  std::vector<int> valid_pairs;
  std::vector<double> reproject_errors;
  std::vector<cv::Mat> intrinsic_matrics;
  std::vector<cv::Mat> distortions;
  std::vector<cv::Size> image_sizes;

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

  for (auto [key, value] : camera_photos) {
    std::cout << "Start calibrating camera #" << key << " with " << value.size()
              << " images" << std::endl;
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    int valid_pair = 0;
    cv::Size image_size;

    for (size_t i = 0; i < value.size(); i++) {
      cv::Mat image = cv::imread(value[i], cv::IMREAD_GRAYSCALE);
      if (image.empty()) {
        continue;
      }
      if (image_size.empty()) {
        image_size = image.size();
      }

      std::vector<cv::Point2f> corners;
      if (!FindCorners(image, pattern_size, corners)) {
        continue;
      }
      object_points.push_back(obj_template);
      image_points.push_back(std::move(corners));
      valid_pair++;
    }

    if (valid_pair < 5) {
      std::cerr << "Need at least 5 valid paris -- found only " << valid_pair
                << "\n"
                << "Skip calibration for this camera #" << key << std::endl;
      continue;
    }
    std::cout << "Found " << valid_pair << " pairs for camera #" << key
              << std::endl;

    cv::Mat intrinsic_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distortion;

    double rms = cv::calibrateCamera(
        object_points, image_points, image_size, intrinsic_matrix, distortion,
        cv::noArray(), cv::noArray(), cv::CALIB_FIX_ASPECT_RATIO);

    std::cout << "RMS for camera #" << key << ": " << rms << std::endl;
    camera_serials.push_back(key);
    valid_pairs.push_back(valid_pair);
    reproject_errors.push_back(rms);
    intrinsic_matrics.push_back(std::move(intrinsic_matrix));
    distortions.push_back(std::move(distortion));
    image_sizes.push_back(image_size);
  }

  auto it = camera_photos.begin();
  const std::string serial1 = it->first;
  const std::vector<std::string> paths1 = it->second;
  ++it;
  const std::string serial2 = it->first;
  const std::vector<std::string> paths2 = it->second;

  std::cout << "Camera 1  serial=" << serial1 << "  images=" << paths1.size()
            << "\n";
  std::cout << "Camera 2  serial=" << serial2 << "  images=" << paths2.size()
            << "\n";
  std::cout << "Chessboard pattern: " << args.pattern_cols << "x"
            << args.pattern_rows << "  square=" << args.square_size
            << " mm\n\n";

  // ------------------------------------------------------------------
  // Save results
  // ------------------------------------------------------------------
  cv::FileStorage fs(args.output_path, cv::FileStorage::WRITE);
  if (!fs.isOpened()) {
    std::cerr << "Cannot open output file: " << args.output_path << "\n";
    return 1;
  }

  fs << "pattern_cols" << args.pattern_cols;
  fs << "pattern_rows" << args.pattern_rows;
  fs << "square_size_mm" << args.square_size;

  fs << "cameras" << "{";

  for (size_t i = 0; i < camera_serials.size(); i++) {
    cv::Size image_size = image_sizes[i];
    fs << camera_serials[i] << "{";
    {
      fs << "image_width" << image_size.width;
      fs << "image_height" << image_size.height;
      fs << "valid_pairs" << valid_pairs[i];
      fs << "intrinsic_matrix" << intrinsic_matrics[i];
      fs << "distortion" << distortions[i];
      fs << "reproject_error" << reproject_errors[i];
    }
    fs << "}";
  }

  fs << "}";

  fs.release();

  std::cout << "Calibration results saved to: " << args.output_path << "\n";
  return 0;
}
