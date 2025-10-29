#include "impls.h"
#include <iostream>

int main() {
    // 读取示例图片
    cv::Mat image = cv::imread("../assets/ap.png");
    if (image.empty()) {
        std::cerr << "Error: Could not open or find the image." << std::endl;
        return -1;
    }

    // 创建检测器实例
    Detector detector;

    // 处理图像
    detector.process(image);

    // 显示结果
    cv::imshow("Armor Plate Detection", image);
    cv::waitKey(0);

    return 0;
}