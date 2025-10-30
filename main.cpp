#include "impls.h"
#include "utils.h"

int main() {
    // 读取示例图片
    cv::Mat image = cv::imread("../assets/ap.png");

    cv::Mat processed_image = erode_image(image, ERODE_THRESHOLD);
    cv::imshow("Eroded image", processed_image);
    cv::waitKey(0);
    cv::destroyAllWindows();

    cv::Mat contour_image = image.clone();
    auto armor_points = contours_connect(processed_image, CONTOUR_AREA_THRESHOLD, contour_image);
    cv::imshow("Connected Contours", contour_image);
    cv::waitKey(0);
    cv::destroyAllWindows();

    if (!armor_points.empty()) {
        cv::Mat pnp_image = image.clone();
        estimatePosePnP(armor_points, pnp_image);
        cv::imshow("PnP Pose", pnp_image);
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    return 0;
}