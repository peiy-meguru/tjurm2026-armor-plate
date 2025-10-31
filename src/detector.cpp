#include "impls.h"

cv::Mat erode_image(const cv::Mat& src_erode, int threshold_value) {
    cv::Mat dst;

    cv::Mat gray;
    cv::cvtColor(src_erode, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, gray, threshold_value, 255, cv::THRESH_BINARY);
    cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ERODE_BORDER, ERODE_BORDER));//大小怎么办？猜一猜
    // 可以，最后一战| 这里7x7、5x5无法识别灯条
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
    std::sort(contours.begin(), contours.end(), [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2) {
        return cv::contourArea(c1) > cv::contourArea(c2);
    });

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
    // 连成多边形
    std::vector<cv::Point> poly_int;
    for(const auto& p : poly) {
        poly_int.push_back(p);
    }
    draw_contours.push_back(poly_int);
    cv::polylines(image, draw_contours, true, cv::Scalar(0, 255, 0), 2);
    for (const auto &p : poly) cv::circle(image, p, 3, cv::Scalar(0, 0, 255), -1);

    return poly;
}

//内参[9.28130989e+02,0,3.77572945e+02,0,9.30138391e+02,2.83892859e+02,0,0,1.0000]
//畸变[-2.54433647e-01, 5.69431382e-01, 3.65405229e-03, -1.09433818e-03, -1.33846840e+00]
int drawAxes(cv::Mat& image, cv::InputArray rvec, cv::InputArray tvec, cv::InputArray cameraMatrix, cv::InputArray distCoeffs, float length) {
    std::vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(0, 0, 0));
    axisPoints.push_back(cv::Point3f(length, 0, 0));
    axisPoints.push_back(cv::Point3f(0, length, 0)); // Y-axis points downwards in object coordinates
    axisPoints.push_back(cv::Point3f(0, 0, length));
    std::vector<cv::Point2f> imagePoints;
    cv::projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);

    cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 3);//x->r,y->g,z->b
    cv::line(image, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 3);
    cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3);

    return 0;
}

int estimatePosePnP(const std::vector<cv::Point2f>& image_points, cv::Mat& image) {
    if (image_points.size() != 4) {
        return -1;
    }

    // AI说假设装甲板间距这样，我也不好说 (135mm x 55mm)
    // 顺序匹配上: lt, rt, rb, lb
    std::vector<cv::Point3f> object_points;
    float half_width = 135.0 / 2 / 1000.0;
    float half_height = 55.0 / 2 / 1000.0;
    // The Y-axis of the object coordinate system points downwards, so the height is negative.
    object_points.push_back(cv::Point3f(-half_width, -half_height, 0));   // lt
    object_points.push_back(cv::Point3f(half_width, -half_height, 0));    // rt
    object_points.push_back(cv::Point3f(half_width, half_height, 0));     // rb
    object_points.push_back(cv::Point3f(-half_width, half_height, 0));    // lb

    // 内参部分
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 9.28130989e+02, 0, 3.77572945e+02, 0, 9.30138391e+02, 2.83892859e+02, 0, 0, 1.0000);
    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << -2.54433647e-01, 5.69431382e-01, 3.65405229e-03, -1.09433818e-03, -1.33846840e+00);

    cv::Mat rvec, tvec;
    // 求位姿
    cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

    // 调用坐标轴绘制函数
    drawAxes(image, rvec, tvec, camera_matrix, dist_coeffs, 0.1);

    return 0;
}

// 视频处理函数，要求：1.读取视频文件（此处为../assets/example.avi）。2.分割每一帧图像并进行二值化处理、装甲板识别、位姿计算。3.用两个窗口分别显示装甲板四个端点练成的四边形、位姿坐标轴绘制结果。
void process_video(const std::string& video_path) {
    cv::VideoCapture cap(video_path);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video file " << video_path << std::endl;
        return;
    }

    cv::Mat frame;
    while (cap.read(frame)) {
        if (frame.empty()) {
            break;
        }

        cv::Mat processed_image = erode_image(frame, ERODE_THRESHOLD);

        cv::Mat contour_image = frame.clone();
        auto armor_points = contours_connect(processed_image, CONTOUR_AREA_THRESHOLD, contour_image);
        cv::imshow("Connected Contours", contour_image);

        if (!armor_points.empty()) {
            cv::Mat pnp_image = frame.clone();
            estimatePosePnP(armor_points, pnp_image);
            cv::imshow("PnP Pose", pnp_image);
        } else {
            // 如果未找到装甲板，则在PnP窗口中显示原始帧
            cv::imshow("PnP Pose", frame);
        }

        if (cv::waitKey(60) >= 0) {
            break;
        }
    }
    cv::waitKey(0);
    cv::destroyAllWindows();
}