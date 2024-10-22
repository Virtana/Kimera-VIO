#pragma once
#include <opencv2/opencv.hpp>

class ColorUtils {
 public:
  static cv::Scalar apricot() { return cv::Scalar(177, 206, 251); }

  static cv::Scalar black() { return cv::Scalar(0, 0, 0); }

  static cv::Scalar blue() { return cv::Scalar(255, 0, 0); }

  static cv::Scalar brown() { return cv::Scalar(42, 42, 165); }

  static cv::Scalar gray() { return cv::Scalar(128, 128, 128); }

  static cv::Scalar green() { return cv::Scalar(0, 255, 0); }

  static cv::Scalar purple() { return cv::Scalar(128, 0, 128); }

  static cv::Scalar pink() { return cv::Scalar(203, 192, 255); }

  static cv::Scalar red() { return cv::Scalar(0, 0, 255); }

  static cv::Scalar yellow() { return cv::Scalar(0, 255, 255); }

  static cv::Scalar white() { return cv::Scalar(255, 255, 255); }

  /**
   * Maps an input h from a value between 0.0 and 1.0 into a rainbow. Copied from
   * OctomapProvider in octomap. Copied from voxblox itself.
   */
  static inline cv::Scalar rainbowColorMap(double h) {
    cv::Scalar color;
    // blend over HSV-values (more colors)

    double s = 1.0;
    double v = 1.0;

    h -= floor(h);
    h *= 6;
    int i;
    double m, n, f;

    i = floor(h);
    f = h - i;
    if (!(i & 1)) f = 1 - f;  // if i is even
    m = v * (1 - s);
    n = v * (1 - s * f);

    switch (i) {
        case 6:
        case 0:
        color = cv::Scalar(255 * m, 255 * n, 255 * v, 255);
        break;
        case 1:
        color = cv::Scalar(255 * m, 255 * v, 255 * n, 255);
        break;
        case 2:
        color = cv::Scalar(255 * n, 255 * v, 255 * m, 255);
        break;
        case 3:
        color = cv::Scalar(255 * v, 255 * n, 255 * m, 255);
        break;
        case 4:
        color = cv::Scalar(255 * v, 255 * m, 255 * n, 255);
        break;
        case 5:
        color = cv::Scalar(255 * n, 255 * m, 255 * v, 255);
        break;
        default:
        color = cv::Scalar(127, 127, 255, 255);
        break;
    }

    return color;
  }

  // Maps an input h from a value between 0.0 and 1.0 into a grayscale color.
  static inline cv::Scalar grayColorMap(double h) {
    auto x = round(h * 255);
    return cv::Scalar(x, x, x);
  }

  static inline cv::Scalar randomColor() {
    return cv::Scalar(rand() % 256, rand() % 256, rand() % 256, 255);
  }

};
