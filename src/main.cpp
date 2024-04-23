#include <fstream>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <string>
#include <vector>

cv::Mat getLane(const cv::Mat& frame, const cv::Mat& roi_mask)
{
	cv::Mat edge;
	Canny(frame, edge, 195, 255);

	cv::Mat blur;
	GaussianBlur(edge, blur, cv::Size(3, 3), 0);

	cv::Mat roi;
	blur.copyTo(roi, roi_mask);
	cv::rectangle(roi, cv::Rect(228, 397, 188, 83), cv::Scalar(0), -1); // lidar_mask

	std::vector<cv::Vec4i> lines;
	HoughLinesP(roi, lines, 1, CV_PI / 180, 1, 50, 5);

	// Draw lines
	for (size_t i = 0; i < lines.size(); i++) {
		cv::Vec4i l = lines[i];
		cv::line(roi, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255), 2, cv::LINE_AA);
	}

	return roi;
}
std::vector<int>getPos(const cv::Mat& roi)
{
	const uchar* offset = roi.ptr<uchar>(400);
	int cols = roi.cols;
	int half_cols = roi.cols / 2;

	//lpos
	std::vector<int> lpos;
	for (int i = 10; i < half_cols; ++i) {
		if (offset[i] == 255) {
			lpos.push_back(i);
		}
	}

	int lpos_size = lpos.size();
	int lpos_x = lpos_size > 0 ? (lpos[0] + lpos[lpos_size - 1]) / 2 : 0; // (lposl + lposr) / 2

	//rpos
	std::vector<int> rpos;
	for (int n = half_cols; n < cols; ++n) {
		if (offset[n] == 255) {
			rpos.push_back(n);
		}
	}

	int rpos_size = rpos.size();
	int rpos_x = rpos_size > 0 ? (rpos[0] + rpos[rpos_size - 1]) / 2 : cols; // (rposl + rposr) / 2

	return{ lpos_x , rpos_x };
}
void drawOnFrame(cv::Mat& frame, std::vector<int> pos_coord)
{
	cv::line(frame, cv::Point(0, 400), cv::Point(640, 400), cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

	for (int x : pos_coord) {
		std::string text = "x : " + std::to_string(x) + " y : " + "400";
		cv::circle(frame, cv::Point(x, 400), 4, cv::Scalar(0, 0, 255), 2, -1);
		cv::putText(frame, text, cv::Point(x, 380), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);
	}
}
int main()
{
	cv::VideoCapture cap;
	cap.open("images/Sub_project.avi");

	std::ofstream csv_file("pos_coord.csv");

	if (!csv_file.is_open()) {
		std::cerr << "Can't open the file." << std::endl;
		return -1;
	}

	csv_file << "lpos" << "," << "rpos" << std::endl;

	// Trapezoid ROI
	std::vector<cv::Point> roi_pts(4);
	roi_pts[0] = cv::Point(0, 480); // bottom left
	roi_pts[1] = cv::Point(20, 350); // top left
	roi_pts[2] = cv::Point(620, 350); // top right
	roi_pts[3] = cv::Point(640, 480); // bottom right

	cv::Mat frame;
	int frame_number = 0;

	while (true) {
		cap >> frame;
		++frame_number;

		if (frame.empty()) {
			std::cerr << "Can't open the video!" << std::endl;
			return -1;
		}

		cv::Mat roi_mask = cv::Mat::zeros(frame.size(), CV_8U);
		fillPoly(roi_mask, roi_pts, cv::Scalar(255));

		cv::Mat roi = getLane(frame, roi_mask);

		std::vector<int>pos_coord = getPos(roi);

		drawOnFrame(frame, pos_coord);

		// Write Csv
		if (frame_number % 30 == 0) {
			csv_file << pos_coord[0] << "," << pos_coord[1] << std::endl;
		}

		//cout << "x1 : " << pos_coord[0] << " x2 : " << pos_coord[1] << endl;

		imshow("src", frame);
		imshow("roi", roi);

		if (cv::waitKey(10) == 27) break;

	}

	csv_file.close();
	cap.release();

}
