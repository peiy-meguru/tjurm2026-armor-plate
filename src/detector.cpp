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
/*
 * To do:
 * 基于OpenCV的装甲板检测项目，原理：
 * 1.首先找到灯条，然后找到灯条端点，对灯条进行匹配，获得装甲板四个灯条端点
 *
 * 目标效果：
 * 将四个端点按顺序连线
 * 对装甲板识别到的点
 * 2.进行 PnP 解算，确定其位姿。参数已经在上方给出，此处重复：
 * 内参 [9.28130989e+02,0,3.77572945e+02,0,9.30138391e+02,2.83892859e+02,0,0,1.0000]
 * 畸变 [-2.54433647e-01, 5.69431382e-01, 3.65405229e-03, -1.09433818e-03, -1.33846840e+00]
*/

Detector::Detector() {
    // 内参
    camera_matrix = (cv::Mat_<double>(3, 3) << 9.28130989e+02, 0, 3.77572945e+02, 0, 9.30138391e+02, 2.83892859e+02, 0, 0, 1.0000);
    // 畸变
    dist_coeffs = (cv::Mat_<double>(1, 5) << -2.54433647e-01, 5.69431382e-01, 3.65405229e-03, -1.09433818e-03, -1.33846840e+00);
}

void Detector::process(cv::Mat& frame) {
    std::vector<LightBar> light_bars;
    findLightBars(frame, light_bars);

    std::vector<Armor> armors;
    matchArmors(light_bars, armors);

    for (auto& armor : armors) {
        solvePnP(armor);
    }

    drawResult(frame, armors);
}

void Detector::findLightBars(const cv::Mat& frame, std::vector<LightBar>& light_bars) {
    cv::Mat gray, binary;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, 180, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        if (contour.size() < 5) continue;
        cv::RotatedRect rotated_rect = cv::minAreaRect(contour);
        LightBar light_bar(rotated_rect);

        // 筛选灯条
        if (light_bar.angle > 45 && light_bar.angle < 135) continue; // 角度筛选
        if (light_bar.height / light_bar.width < 1.5) continue; // 长宽比筛选

        light_bars.push_back(light_bar);
    }
}

void Detector::matchArmors(const std::vector<LightBar>& light_bars, std::vector<Armor>& armors) {
    for (size_t i = 0; i < light_bars.size(); ++i) {
        for (size_t j = i + 1; j < light_bars.size(); ++j) {
            const auto& l1 = light_bars[i];
            const auto& l2 = light_bars[j];

            // 筛选匹配的灯条
            float angle_diff = std::abs(l1.angle - l2.angle);
            if (angle_diff > 10) continue; // 角度差

            float height_diff_ratio = std::abs(l1.height - l2.height) / std::max(l1.height, l2.height);
            if (height_diff_ratio > 0.2) continue; // 高度差

            float distance = cv::norm(l1.center - l2.center);
            float avg_height = (l1.height + l2.height) / 2;
            if (distance / avg_height > 5 || distance / avg_height < 1.5) continue; // 距离与高度比

            armors.emplace_back(l1, l2);
        }
    }
}

void Detector::solvePnP(Armor& armor) {
    // 小装甲板实际尺寸
    const float small_armor_width = 135;
    const float small_armor_height = 55;
    // 大装甲板实际尺寸
    const float large_armor_width = 225;
    const float large_armor_height = 55;

    float armor_width = (cv::norm(armor.left_light.center - armor.right_light.center) / ((armor.left_light.height + armor.right_light.height)/2.0) > 3.0) ? large_armor_width : small_armor_width;
    float armor_height = (cv::norm(armor.left_light.center - armor.right_light.center) / ((armor.left_light.height + armor.right_light.height)/2.0) > 3.0) ? large_armor_height : small_armor_height;


    std::vector<cv::Point3f> object_points = {
        {-armor_width / 2, -armor_height / 2, 0},
        {armor_width / 2, -armor_height / 2, 0},
        {armor_width / 2, armor_height / 2, 0},
        {-armor_width / 2, armor_height / 2, 0}
    };

    cv::Point2f vertices[4];
    armor.left_light.rect.points(vertices);
    cv::Point2f tl = vertices[1];
    cv::Point2f bl = vertices[0];
    armor.right_light.rect.points(vertices);
    cv::Point2f tr = vertices[2];
    cv::Point2f br = vertices[3];

    if (armor.left_light.rect.size.width > armor.left_light.rect.size.height) {
        tl = vertices[2];
        bl = vertices[1];
    }
    if (armor.right_light.rect.size.width > armor.right_light.rect.size.height) {
        tr = vertices[3];
        br = vertices[2];
    }

    armor.armor_pts = {bl, br, tr, tl};

    cv::Mat rvec, tvec;
    cv::solvePnP(object_points, armor.armor_pts, camera_matrix, dist_coeffs, rvec, tvec);

    // 在drawResult中绘制坐标轴
    armor.rvec = rvec;
    armor.tvec = tvec;
}

void Detector::drawResult(cv::Mat& frame, const std::vector<Armor>& armors) {
    for (const auto& armor : armors) {
        // 绘制装甲板轮廓
        for (size_t i = 0; i < armor.armor_pts.size(); ++i) {
            cv::line(frame, armor.armor_pts[i], armor.armor_pts[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }

        // 绘制坐标轴
        if (!armor.rvec.empty() && !armor.tvec.empty()) {
            std::vector<cv::Point3f> axis_points = {
                {0, 0, 0}, {100, 0, 0}, {0, 100, 0}, {0, 0, 100} // 修正Z轴方向
            };
            std::vector<cv::Point2f> image_points;
            cv::projectPoints(axis_points, armor.rvec, armor.tvec, camera_matrix, dist_coeffs, image_points);

            // 使用 arrowedLine 绘制带箭头的加粗坐标轴
            cv::arrowedLine(frame, image_points[0], image_points[1], cv::Scalar(0, 0, 255), 3); // X-axis in red
            cv::arrowedLine(frame, image_points[0], image_points[2], cv::Scalar(0, 255, 0), 3); // Y-axis in green
            cv::arrowedLine(frame, image_points[0], image_points[3], cv::Scalar(255, 0, 0), 3); // Z-axis in blue
        }
    }
}