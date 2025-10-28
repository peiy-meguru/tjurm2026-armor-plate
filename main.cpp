#include "impls.h"
#include <iostream>

// --- 通过取消注释以下其中一行来选择模式 ---
#define USE_VIDEO 
// #define USE_IMAGE

int main() {
    ArmorDetector detector;

#ifdef USE_VIDEO
    // --- 使用视频文件 ---
    // 请将 "path/to/your/video.mp4" 替换为您的视频文件路径
    cv::VideoCapture cap("path/to/your/video.mp4"); 

    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video file." << std::endl;
        return -1;
    }

    cv::Mat frame;
    while (cap.read(frame)) {
        if (frame.empty()) {
            std::cout << "Video has ended." << std::endl;
            break;
        }

        detector.detect(frame);
        cv::imshow("Armor Plate Detection", frame);

        // 等待30毫秒，如果按下ESC键则退出
        if (cv::waitKey(30) == 27) { 
            break;
        }
    }
#endif

#ifdef USE_IMAGE
    // --- 使用图片文件 ---
    // 请将 "path/to/your/image.jpg" 替换为您的图片文件路径
    cv::Mat image = cv::imread("path/to/your/image.jpg");
    if (image.empty()) {
        std::cerr << "Error: Could not open image file." << std::endl;
        return -1;
    }

    detector.detect(image);
    cv::imshow("Armor Plate Detection", image);
    cv::waitKey(0); // 无限期等待按键
#endif

    return 0;
}

