#include "impls.h"
#include "utils.h"

int main() {
    // 读取示例图片
    cv::Mat image = cv::imread("../assets/ap.png");

    cv::Mat processed_image = erode_image(image, ERODE_THRESHOLD);
    cv::imshow("Eroded image", processed_image);
    cv::waitKey(0);
    cv::destroyAllWindows();

    std::vector<cv::RotatedRect> rrects = contours_connect(processed_image, CONTOUR_AREA_THRESHOLD, image);
    cv::imshow("Result image", image);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}