#include "impls.h"

cv::Mat show_rectangle(const cv::RotatedRect& rrect, int rows, int cols) {
    cv::Mat empty = cv::Mat::zeros(cv::Size(cols, rows), CV_8UC3);
    cv::Point2f pnts[4];
    rrect.points(pnts);
    for (int j = 0; j < 4; j++)
        line(empty, pnts[j], pnts[(j + 1) % 4], { 255, 255, 255 });
    return empty;
}