#include <opencv2/opencv.hpp>
#include <vector>

#include <iostream>

#define ERODE_THRESHOLD 170
#define CONTOUR_AREA_THRESHOLD 1
//经过测试，70即可去除字符“3”，150基本只保留灯条，到达240仍然有灯条保留

cv::Mat erode_image(const cv::Mat& src, int threshold_value);

std::vector<cv::Point2f> contours_connect(const cv::Mat& src, int threshold_value, const cv::Mat& image);
void estimatePosePnP(const std::vector<cv::Point2f>& image_points, cv::Mat& image);
void drawAxes(cv::Mat& image, cv::InputArray rvec, cv::InputArray tvec, cv::InputArray cameraMatrix, cv::InputArray distCoeffs, float length);

cv::Mat show_rectangle(const cv::RotatedRect& rrect, int rows, int cols);

//int bars_connected(cv::Mat& image, const cv::Mat& eroded);