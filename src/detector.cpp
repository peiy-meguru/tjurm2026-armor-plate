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

std::vector<cv::Point2f> contours_connect(const cv::Mat& src, int threshold_value, const cv::Mat& image) {
    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(src, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.size() < 2) {
        return {}; // Not enough contours found
    }
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

    std::vector<cv::Point2f> poly;
    poly.push_back(lt);
    poly.push_back(rt);
    poly.push_back(rb);
    poly.push_back(lb);

    // 绘制并标注
    std::vector<std::vector<cv::Point>> draw_contours;
    // convert Point2f to Point for polylines
    std::vector<cv::Point> poly_int;
    for(const auto& p : poly) {
        poly_int.push_back(p);
    }
    draw_contours.push_back(poly_int);
    cv::polylines(image, draw_contours, true, cv::Scalar(0, 255, 0), 2);
    for (const auto &p : poly) cv::circle(image, p, 3, cv::Scalar(0, 0, 255), -1);

    return poly;
}

//estimatePosePnP
//内参[9.28130989e+02,0,3.77572945e+02,0,9.30138391e+02,2.83892859e+02,0,0,1.0000]
//畸变[-2.54433647e-01, 5.69431382e-01, 3.65405229e-03, -1.09433818e-03, -1.33846840e+00]
void drawAxes(cv::Mat& image, cv::InputArray rvec, cv::InputArray tvec, cv::InputArray cameraMatrix, cv::InputArray distCoeffs, float length) {
    std::vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(0, 0, 0));
    axisPoints.push_back(cv::Point3f(length, 0, 0));
    axisPoints.push_back(cv::Point3f(0, length, 0));
    axisPoints.push_back(cv::Point3f(0, 0, length));
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

    cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 3); // X-axis in red
    cv::line(image, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 3); // Y-axis in green
    cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3); // Z-axis in blue
}

void estimatePosePnP(const std::vector<cv::Point2f>& image_points, cv::Mat& image) {
    if (image_points.size() != 4) {
        return;
    }

    // 3D model points of the armor plate in world coordinates (in meters)
    // Assuming a small armor plate (135mm x 55mm)
    // The order should match the image_points: lt, rt, rb, lb
    std::vector<cv::Point3f> object_points;
    float half_width = 135.0 / 2 / 1000.0;
    float half_height = 55.0 / 2 / 1000.0;
    object_points.push_back(cv::Point3f(-half_width, half_height, 0));    // lt
    object_points.push_back(cv::Point3f(half_width, half_height, 0));     // rt
    object_points.push_back(cv::Point3f(half_width, -half_height, 0));    // rb
    object_points.push_back(cv::Point3f(-half_width, -half_height, 0));   // lb

    // Camera internals
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 9.28130989e+02, 0, 3.77572945e+02, 0, 9.30138391e+02, 2.83892859e+02, 0, 0, 1.0000);
    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << -2.54433647e-01, 5.69431382e-01, 3.65405229e-03, -1.09433818e-03, -1.33846840e+00);

    cv::Mat rvec, tvec;
    // Solve for pose
    cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

    // Draw coordinate axes
    drawAxes(image, rvec, tvec, camera_matrix, dist_coeffs, 0.1);
}