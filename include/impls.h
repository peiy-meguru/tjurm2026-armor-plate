#include <opencv2/opencv.hpp>
#include <vector>

// 代表一个灯条
struct LightBar {
    cv::RotatedRect rect; // 灯条的旋转矩形
    double angle;         // 角度
    double height;        // 高度
};

// 代表一个装甲板
struct ArmorPlate {
    std::vector<cv::Point2f> vertices; // 装甲板的四个顶点
    cv::Point2f center;                // 中心点
};

class ArmorDetector {
public:
    ArmorDetector();
    void detect(cv::Mat& frame);

private:
    void preprocess(const cv::Mat& frame, cv::Mat& out);
    std::vector<LightBar> findLightBars(const cv::Mat& binary);
    std::vector<ArmorPlate> matchArmorPlates(const std::vector<LightBar>& light_bars);
    void solvePnP(const std::vector<ArmorPlate>& armor_plates, cv::Mat& frame);

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    std::vector<cv::Point3f> small_armor_points;
};
