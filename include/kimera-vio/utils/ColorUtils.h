#pragma once

#include <opencv2/core/core.hpp>

namespace VIO {

/* ColorUtils is used instead of cv::viz::Color in non-visualizer classes to 
   facilitate building without the visualizer. Color values are taken from 
   https://github.com/apc-llc/opencv-2.4.10/blob/master/modules/viz/include/opencv2/viz/types.hpp#L191
*/
namespace ColorUtils {
  inline cv::Scalar Apricot() { return cv::Scalar(177, 206, 251); }
  inline cv::Scalar Black() { return cv::Scalar(0, 0, 0); }
  inline cv::Scalar Blue() { return cv::Scalar(255, 0, 0); }    
  inline cv::Scalar Brown() { return cv::Scalar(0, 75, 150); }
  inline cv::Scalar Green() { return cv::Scalar(0, 255, 0); }
  inline cv::Scalar Pink() { return cv::Scalar(203, 192, 255); }
  inline cv::Scalar Purple() { return cv::Scalar(128, 0, 128); }
  inline cv::Scalar Red() { return cv::Scalar(0, 0, 255); }
  inline cv::Scalar White() { return cv::Scalar(255, 255, 255); }

  inline cv::Vec3b ScalarToVec3b(cv::Scalar value) {
    return cv::Vec3b(static_cast<uchar>(value[0]), 
                     static_cast<uchar>(value[1]),
                     static_cast<uchar>(value[2]));
    
  }
} //ColorUtils namespace
} // VIO namespace
