// Copyright (C) 2023 Grepp CO.
// All rights reserved.

/**
 * @file LaneDetector.cpp
 * @author Jeongmin Kim
 * @author Jeongbin Yim
 * @brief lane detector class source file
 * @version 2.1
 * @date 2023-10-13
 */

#include <numeric>
#include "LaneKeepingSystem/LaneDetector.hpp"


namespace Xycar {
template <typename PREC>
void LaneDetector<PREC>::setConfiguration(const YAML::Node& config)
{
    mImageWidth = config["IMAGE"]["WIDTH"].as<int32_t>();
    mImageHeight = config["IMAGE"]["HEIGHT"].as<int32_t>();
    mDebugging = config["DEBUG"].as<bool>();
}

template <typename PREC>
void LaneDetector<PREC>::imageProcessing(const cv::Mat& mFrame)
{
    warpcase = cv::getPerspectiveTransform(roi_pts, Brideyeview_pts);
    cv::warpPerspective(mFrame, warp, warpcase, mFrame.size());
    cv::GaussianBlur(warp, blur, gblur_size, 0);
    cv::cvtColor(blur, hsv_img, cv::COLOR_BGR2HSV);
	cv::inRange(hsv_img, lower_black, upper_black, black_mask);

    sliding_window_lane_recognition(black_mask);
}

template <typename PREC>
void LaneDetector<PREC>::sliding_window_lane_recognition(const cv::Mat& binary_image)
{
    int window_height = binary_image.rows / nwindows;
    int window_width = 2 * margin;

    // To get coordinates that are could to be Lane
    std::vector<cv::Point> left_lane_inds;
    std::vector<cv::Point> right_lane_inds;

    // To get Lane coordinates to get Lane incline
    std::vector<int> lpos_points = {0,0,0,0,1,0,0,0,1};
    std::vector<int> rpos_points = {640,640,640,640,641,640,640,640,641};

    cv::Mat histogram;
    cv::reduce(binary_image.rowRange(0, binary_image.rows), histogram, 0, cv::REDUCE_SUM, CV_32S);

    int midpoint = histogram.cols / 2;
    // Find the maximum value location in the left and right halves of the histogram
    cv::Point leftLoc, rightLoc;
    cv::minMaxLoc(histogram.colRange(0, midpoint), NULL, NULL, NULL, &leftLoc);
    cv::minMaxLoc(histogram.colRange(midpoint, histogram.cols), NULL, NULL, NULL, &rightLoc);
    int leftx_base = leftLoc.x;
    int rightx_base = rightLoc.x > 0 ? rightLoc.x + midpoint : mImageWidth;

    int leftx_current = leftx_base;
    int rightx_current = rightx_base;

    for (int window = 0; window < nwindows; ++window) {
        int win_y_low = binary_image.rows - (window + 1) * window_height;
        int win_y_high = binary_image.rows - window * window_height;

        int win_xleft_low = leftx_current - margin;
        int win_xleft_high = leftx_current + margin;
        int win_xright_low = rightx_current - margin;
        int win_xright_high = rightx_current + margin;

        std::vector<cv::Point> nonzero;
        cv::findNonZero(binary_image(cv::Rect(0, win_y_low, binary_image.cols, window_height)), nonzero);

        for (cv::Point p : nonzero) {
            // Check if the point is within the window bounds
            if (win_xleft_low <= p.x && p.x < win_xleft_high) {
                left_lane_inds.push_back(p);
            }

            if (win_xright_low <= p.x && p.x < win_xright_high) {
                right_lane_inds.push_back(p);
            }
        }

        // Update the current x position based on the mean of the points found
        if (left_lane_inds.size() > minpix) {
            leftx_current = cv::mean(cv::Mat(left_lane_inds))[0];
            lpos_points[window] = leftx_current;
        }
        
        if (right_lane_inds.size() > minpix) {
            rightx_current = cv::mean(cv::Mat(right_lane_inds))[0];
            rpos_points[window] = rightx_current;
        }

        lpos += leftx_current;
        rpos += rightx_current;

        left_lane_inds.clear();
        right_lane_inds.clear();
    }
    int lpos_size = lpos_points.size();
    int rpos_size = rpos_points.size();

    // Get left & right lane coordinates
    lpos = lpos_points[1];
    rpos = rpos_points[1];
    
    // Get inlcine of left & right lane
    double lpos_inclination1 = 3*lpos_size / (lpos_points[4]-lpos_points[2]);
    double lpos_inclination2 = 3*lpos_size / (lpos_points[8]-lpos_points[5]);
    double rpos_inclination1 = 3*rpos_size / (rpos_points[4]-rpos_points[2]);
    double rpos_inclination2 = 3*rpos_size / (rpos_points[8]-rpos_points[4]);
    
    double lpos_inclination=lpos_inclination1+lpos_inclination2;
    double rpos_inclination=rpos_inclination1+rpos_inclination2;

    // if lpos and rpos have 530 pixel away, it is straight lane.
    // else correct the value of lpos & rpos
    if (abs(rpos - lpos) < straight_lane_interval) {
        if (0 < rpos_inclination && rpos_inclination < curve_lane_incline) {
            lpos = rpos;
            rpos = mImageWidth;
        }

        else if ((-1)*curve_lane_incline < lpos_inclination && lpos_inclination < 0) {
            rpos = lpos;
            lpos = 0;
        }

        if(0 < lpos_inclination && lpos_inclination < curve_lane_incline ){
            rpos = mImageWidth;
        }

        else if((-1)*curve_lane_incline < rpos_inclination && rpos_inclination < 0){
            lpos = 0;
        }
    }

    mean_of_lane = (lpos + rpos) / 2;
    error_calculated = mean_of_lane - center_point;

    lpos = 0;
    rpos = 0;
}

template class LaneDetector<float>;
template class LaneDetector<double>;
} // namespace Xycar
