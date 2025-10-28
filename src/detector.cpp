#include "impls.h"
#include <iostream>

/*
 * 显然装甲板识别需要几个步骤
    * 1. 如果是视频，则视频分割成图片；如果是图片，直接读取
    * 2. 图像预处理（灰度化二值化等）
    * 3. 边缘检测，收集端点，连线
    * 4. PnP解算，确定位姿
    * 5. 显示坐标轴
    * 注：实例图片
    * 内参 [9.28130989e+02,0,3.77572945e+02,0,9.30138391e+02,2.83892859e+02,0,0,1.0000]
    * 畸变 [-2.54433647e-01, 5.69431382e-01, 3.65405229e-03, -1.09433818e-03, -1.33846840e+00]
*/

ArmorDetector::ArmorDetector() {
    // 内参矩阵
    camera_matrix = (cv::Mat_<double>(3, 3) << 9.28130989e+02, 0, 3.77572945e+02, 0, 9.30138391e+02, 2.83892859e+02, 0, 0, 1.0000);
    // 畸变系数
    dist_coeffs = (cv::Mat_<double>(1, 5) << -2.54433647e-01, 5.69431382e-01, 3.65405229e-03, -1.09433818e-03, -1.33846840e+00);

    // 小装甲板的3D坐标 (宽度: 135mm, 高度: 55mm)
    double half_width = 135.0 / 2.0;
    double half_height = 55.0 / 2.0;
    small_armor_points.push_back(cv::Point3f(-half_width, -half_height, 0));
    small_armor_points.push_back(cv::Point3f(half_width, -half_height, 0));
    small_armor_points.push_back(cv::Point3f(half_width, half_height, 0));
    small_armor_points.push_back(cv::Point3f(-half_width, half_height, 0));
}

void ArmorDetector::preprocess(const cv::Mat& frame, cv::Mat& out) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, out, 150, 255, cv::THRESH_BINARY);
}

std::vector<LightBar> ArmorDetector::findLightBars(const cv::Mat& binary) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<LightBar> light_bars;
    for (const auto& contour : contours) {
        if (contour.size() < 5) continue;

        cv::RotatedRect rotated_rect = cv::minAreaRect(contour);
        double area_ratio = cv::contourArea(contour) / (rotated_rect.size.width * rotated_rect.size.height);

        if (area_ratio < 0.5) continue; // 过滤掉空心的轮廓

        double angle = rotated_rect.angle;
        double height = std::max(rotated_rect.size.width, rotated_rect.size.height);
        double width = std::min(rotated_rect.size.width, rotated_rect.size.height);
        double aspect_ratio = height / width;

        if (aspect_ratio < 2.0 || aspect_ratio > 10.0) continue;

        LightBar light_bar;
        light_bar.rect = rotated_rect;
        light_bar.height = height;
        if (rotated_rect.size.width > rotated_rect.size.height) {
            light_bar.angle = angle + 90;
        } else {
            light_bar.angle = angle;
        }
        light_bars.push_back(light_bar);
    }
    return light_bars;
}

std::vector<ArmorPlate> ArmorDetector::matchArmorPlates(const std::vector<LightBar>& light_bars) {
    std::vector<ArmorPlate> armor_plates;
    for (size_t i = 0; i < light_bars.size(); ++i) {
        for (size_t j = i + 1; j < light_bars.size(); ++j) {
            const auto& bar1 = light_bars[i];
            const auto& bar2 = light_bars[j];

            // 角度差
            double angle_diff = std::abs(bar1.angle - bar2.angle);
            if (angle_diff > 10) continue;

            // 高度差
            double height_diff_ratio = std::abs(bar1.height - bar2.height) / std::max(bar1.height, bar2.height);
            if (height_diff_ratio > 0.2) continue;

            // 宽度比
            double distance = cv::norm(bar1.rect.center - bar2.rect.center);
            double avg_height = (bar1.height + bar2.height) / 2.0;
            double distance_height_ratio = distance / avg_height;
            if (distance_height_ratio < 1.5 || distance_height_ratio > 4.5) continue;

            ArmorPlate armor;
            armor.center = (bar1.rect.center + bar2.rect.center) / 2.0;

            cv::Point2f bar1_pts[4], bar2_pts[4];
            bar1.rect.points(bar1_pts);
            bar2.rect.points(bar2_pts);

            // 简单的顶点排序逻辑
            if (bar1.rect.center.x < bar2.rect.center.x) {
                armor.vertices.push_back(bar1_pts[1]);
                armor.vertices.push_back(bar2_pts[0]);
                armor.vertices.push_back(bar2_pts[3]);
                armor.vertices.push_back(bar1_pts[2]);
            } else {
                armor.vertices.push_back(bar2_pts[1]);
                armor.vertices.push_back(bar1_pts[0]);
                armor.vertices.push_back(bar1_pts[3]);
                armor.vertices.push_back(bar2_pts[2]);
            }
            armor_plates.push_back(armor);
        }
    }
    return armor_plates;
}

void ArmorDetector::solvePnP(const std::vector<ArmorPlate>& armor_plates, cv::Mat& frame) {
    if (armor_plates.empty()) return;

    // 假设我们只处理找到的第一个装甲板
    const auto& armor = armor_plates[0];

    cv::Mat rvec, tvec;
    cv::solvePnP(small_armor_points, armor.vertices, camera_matrix, dist_coeffs, rvec, tvec);

    // 在图像上绘制坐标轴
    std::vector<cv::Point3f> axis_points;
    axis_points.push_back(cv::Point3f(0, 0, 0));
    axis_points.push_back(cv::Point3f(50, 0, 0)); // X轴
    axis_points.push_back(cv::Point3f(0, 50, 0)); // Y轴
    axis_points.push_back(cv::Point3f(0, 0, -50)); // Z轴

    std::vector<cv::Point2f> image_points;
    cv::projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs, image_points);

    cv::line(frame, image_points[0], image_points[1], cv::Scalar(0, 0, 255), 2); // X-Red
    cv::line(frame, image_points[0], image_points[2], cv::Scalar(0, 255, 0), 2); // Y-Green
    cv::line(frame, image_points[0], image_points[3], cv::Scalar(255, 0, 0), 2); // Z-Blue

    // 绘制装甲板轮廓
    for (size_t i = 0; i < 4; ++i) {
        cv::line(frame, armor.vertices[i], armor.vertices[(i + 1) % 4], cv::Scalar(0, 255, 255), 2);
    }
}

void ArmorDetector::detect(cv::Mat& frame) {
    cv::Mat binary_img;
    preprocess(frame, binary_img);

    auto light_bars = findLightBars(binary_img);
    auto armor_plates = matchArmorPlates(light_bars);

    solvePnP(armor_plates, frame);
}
