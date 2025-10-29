#include "impls.h"

cv::Mat erode_image(const cv::Mat& src_erode, int threshold_value) {
    cv::Mat dst;

    cv::Mat gray;
    cv::cvtColor(src_erode, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, gray, threshold_value, 255, cv::THRESH_BINARY);
    cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));//大小怎么办？猜一猜
    cv::erode(gray, dst, erode_kernel);

    return dst;
    // erode用于消除小的白色噪声，减小前景区域
}

std::vector<cv::RotatedRect> contours_connect(const cv::Mat& src, int threshold_value, const cv::Mat& image) {
    std::vector<cv::RotatedRect> res;
    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(src, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::RotatedRect r1 = cv::minAreaRect(contours[0]);//矩形近似化
    cv::RotatedRect r2 = cv::minAreaRect(contours[1]);
    
    //下方有待研究
    cv::RotatedRect leftRect = (r1.center.x < r2.center.x) ? r1 : r2;//识别左右矩阵，毕竟你也不能翻转过来吧
    cv::RotatedRect rightRect = (r1.center.x < r2.center.x) ? r2 : r1;
    
    cv::Point2f lpts[4], rpts[4];
    leftRect.points(lpts);
    rightRect.points(rpts);

    // 分别对左右两个矩形的顶点进行排序，找到四个端点
    std::sort(lpts, lpts + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
    std::sort(rpts, rpts + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });

    cv::Point2f lt = lpts[0].x < lpts[1].x ? lpts[0] : lpts[1];
    cv::Point2f lb = lpts[2].x < lpts[3].x ? lpts[2] : lpts[3];
    cv::Point2f rt = rpts[0].x > rpts[1].x ? rpts[0] : rpts[1];
    cv::Point2f rb = rpts[2].x > rpts[3].x ? rpts[2] : rpts[3];

    std::vector<cv::Point> poly;
    poly.push_back(lt);
    poly.push_back(rt);
    poly.push_back(rb);
    poly.push_back(lb);

    // 绘制并标注
    std::vector<std::vector<cv::Point>> draw_contours;
    draw_contours.push_back(poly);
    cv::polylines(image, draw_contours, true, cv::Scalar(0, 255, 0), 2);
    for (const auto &p : poly) cv::circle(image, p, 3, cv::Scalar(0, 0, 255), -1);

    return res;
}