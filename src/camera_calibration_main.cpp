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
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

struct CameraCalibrationConfig {
  std::string serial;
  cv::Size chessboard_size;
  float square_size_mm;
  std::vector<std::string> photos;
};

// ------------------------------
// yaml parsing
// ------------------------------
static std::map<std::string, CameraCalibrationConfig>
YamlParseConfig(const std::string &path) {
  cv::FileStorage fs(path, cv::FileStorage::READ);
  std::map<std::string, CameraCalibrationConfig> configs;

  if (fs.isOpened()) {
    cv::FileNode cameras_node = fs["cameras"];
    if (!cameras_node.empty() && cameras_node.isMap()) {
      for (auto it = cameras_node.begin(); it != cameras_node.end(); ++it) {
        CameraCalibrationConfig config;
        std::string node_name = (*it).name();

        // Extract serial from node name (e.g. "camera_903223052137" ->
        // "903223052137")
        if (node_name.find("camera_") == 0) {
          config.serial = node_name.substr(7);
        } else {
          config.serial = node_name;
        }

        int cols = 0, rows = 0;
        (*it)["pattern_cols"] >> cols;
        (*it)["pattern_rows"] >> rows;
        config.chessboard_size = cv::Size(cols, rows);

        (*it)["square_size_mm"] >> config.square_size_mm;

        cv::FileNode photos_node = (*it)["photos"];
        if (!photos_node.empty() && photos_node.isSeq()) {
          for (auto pt = photos_node.begin(); pt != photos_node.end(); ++pt) {
            std::string photo_path;
            *pt >> photo_path;
            config.photos.push_back(photo_path);
          }
        }

        configs[config.serial] = config;
      }
    }
  }

  return configs;
}

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
};

static void PrintUsage(const char *prog) {
  std::cerr << "Usage: " << prog << " --config <xml> --output <yaml>\n";
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

  // load configuration from yaml file
  auto configs = YamlParseConfig(args.config_path);

  std::vector<std::string> camera_serials;
  std::vector<int> valid_pairs;
  std::vector<double> reproject_errors;
  std::vector<cv::Mat> intrinsic_matrics;
  std::vector<cv::Mat> distortions;
  std::vector<cv::Size> image_sizes;

  for (auto [key, value] : configs) {
    std::cout << "Start calibrating camera #" << key << " with "
              << value.photos.size() << " images" << std::endl;

    // ------------------------------------------------------------------
    // Build object-point template (the same for every view)
    // ------------------------------------------------------------------
    cv::Size pattern_size(value.chessboard_size.width, value.chessboard_size.height);

    std::vector<cv::Point3f> obj_template;
    obj_template.reserve(pattern_size.width * pattern_size.height);
    for (int r = 0; r < pattern_size.height; ++r) {
      for (int c = 0; c < pattern_size.width; ++c) {
        obj_template.emplace_back(static_cast<float>(c * value.square_size_mm),
                                  static_cast<float>(r * value.square_size_mm), 0.0f);
      }
    }

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    int valid_pair = 0;
    cv::Size image_size;

    for (size_t i = 0; i < value.photos.size(); i++) {
      cv::Mat image = cv::imread(value.photos[i], cv::IMREAD_GRAYSCALE);
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

  // ------------------------------------------------------------------
  // Save results
  // ------------------------------------------------------------------
  cv::FileStorage fs(args.output_path, cv::FileStorage::WRITE);
  if (!fs.isOpened()) {
    std::cerr << "Cannot open output file: " << args.output_path << "\n";
    return 1;
  }

  fs << "cameras" << "{";

  for (size_t i = 0; i < camera_serials.size(); i++) {
    cv::Size image_size = image_sizes[i];
    fs << "camera_" + camera_serials[i] << "{";
    {
      auto& camera_config = configs[camera_serials[i]];
      fs << "pattern_cols" << camera_config.chessboard_size.width;
      fs << "pattern_rows" << camera_config.chessboard_size.height;
      fs << "square_size_mm" << camera_config.square_size_mm;
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
