// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019-24 RealSense, Inc. All Rights Reserved.

#include <iostream>
#include <librealsense2/rs.hpp> // RealSense Cross Platform API

#include <opencv2/opencv.hpp>

// Hello RealSense example demonstrates the basics of connecting to a RealSense
// device and taking advantage of depth data
int main(int argc, char *argv[]) try {

  cv::FileStorage fs("test_realsense_main.yml", cv::FileStorage::WRITE);
  if (!fs.isOpened()) {
    std::cerr << "Cannot open output file: test_realsense_main.yml\n";
    return EXIT_FAILURE;
  }

  fs << "test_value" << 42;
  fs << "K1" << (cv::Mat_<double>(3, 3) << 1000, 0, 640, 0, 1000, 360, 0, 0, 1);
  fs.release();
  

  return EXIT_SUCCESS;
} catch (const rs2::error &e) {
  std::cerr << "RealSense error calling " << e.get_failed_function() << "("
            << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
} catch (const std::exception &e) {
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}
