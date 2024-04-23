#ifndef LANE_DETECTOR_HPP_
#define LANE_DETECTOR_HPP_

#include "opencv2/opencv.hpp"
#include <yaml-cpp/yaml.h>

namespace Xycar {
template <typename PREC>
class LaneDetector final
{
public:
    using Ptr = LaneDetector*;

    static inline const cv::Scalar kRed = {0, 0, 255}; /// Scalar values of Red
    static inline const cv::Scalar kGreen = {0, 255, 0}; /// Scalar values of Green
    static inline const cv::Scalar kBlue = {255, 0, 0}; /// Scalar values of Blue

    LaneDetector(const YAML::Node& config) {setConfiguration(config);}
    void imageProcessing(const cv::Mat& mFrame); // Processing image for line detecting

    int lpos = 0;
    int rpos = 0 ;
    int mean_of_lane = 0;
    int center_point = 320;
    int error_calculated = 0;
    int straight_lane_interval = 530;
    int curve_lane_incline = 5;

private:
    int32_t mImageWidth;
    int32_t mImageHeight;

    cv::Mat warpcase; // Polygon for transform nFrame
    cv::Mat warp; // Apply perspective transform to obtain a bird's-eye view
    cv::Mat blur; // Apply bluring image
    cv::Mat hsv_img; // Apply grayscale image
    cv::Mat black_mask;
    cv::Mat mDebugFrame; /// < The frame for debugging

    cv::Scalar lower_black = cv::Scalar(40,0,0);    // Threshold to binarization birdeyeview image
	cv::Scalar upper_black = cv::Scalar(180, 100, 120);

    void setConfiguration(const YAML::Node& config);
    bool mDebugging;

    void sliding_window_lane_recognition(const cv::Mat& binary_image); // Get lpos & rpos with sliding window 
    void histogram(const cv::Mat& binary); // Calculate & show histogram graph of grayscale image
    void drawRect_warp(const cv::Mat& warpImg, int xleft, int yleft, int xright, int yright, int win_width, int win_height);

    int nwindows = 9; // number of sliding window
    int margin = 25; // Size(rows) of sliding window
    int minpix = 10; // Minimum number of pixels to detect line

    cv::Size gblur_size(7, 7);

    cv::Point2f roi_pts[4] = {cv::Point2f(95, 350), cv::Point2f(20, 400), cv::Point2f(545, 350), cv::Point2f(620, 400)};
    // roi_pts[0] = cv::Point(95, 350); // top left
	// roi_pts[1] = cv::Point(20, 400); // bottom left
	// roi_pts[2] = cv::Point(545, 350); // top right
	// roi_pts[3] = cv::Point(620, 400); // bottom right

    cv::Point2f Brideyeview_pts[4] = {cv::Point2f(0,0) , cv::Point2f(0,480) , cv::Point2f(640,0) , cv::Point2f(640,480)};
    // Brideyeview_pts[0] = cv::Point(0,0);
    // Brideyeview_pts[1] = cv::Point(0,480);
    // Brideyeview_pts[2] = cv::Point(640,0);
    // Brideyeview_pts[3] = cv::Point(640,480);
};
}

#endif // LANE_DETECTOR_HPP_