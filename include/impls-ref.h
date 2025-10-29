#include <opencv2/opencv.hpp>
#include <vector>

// 灯条类
class LightBar {
public:
    LightBar() = default;
    LightBar(const cv::RotatedRect& rect) :
        rect(rect) {
        cv::Point2f vertices[4];
        rect.points(vertices);
        center = rect.center;
        height = std::max(rect.size.width, rect.size.height);
        width = std::min(rect.size.width, rect.size.height);
        angle = rect.angle;
        if (rect.size.width > rect.size.height) {
            angle += 90;
        }
    }

    cv::RotatedRect rect;
    cv::Point2f center;
    float height = 0;
    float width = 0;
    float angle = 0;
};

// 装甲板类
class Armor {
public:
    Armor() = default;
    Armor(const LightBar& l1, const LightBar& l2) {
        if (l1.center.x < l2.center.x) {
            left_light = l1;
            right_light = l2;
        } else {
            left_light = l2;
            right_light = l1;
        }
        center = (left_light.center + right_light.center) / 2;
    }

    LightBar left_light;
    LightBar right_light;
    cv::Point2f center;
    std::vector<cv::Point2f> armor_pts;
    cv::Mat rvec, tvec;
};

// 检测器类
class Detector {
public:
    Detector();
    void process(cv::Mat& frame);

private:
    void findLightBars(const cv::Mat& frame, std::vector<LightBar>& light_bars);
    void matchArmors(const std::vector<LightBar>& light_bars, std::vector<Armor>& armors);
    void solvePnP(Armor& armor);
    void drawResult(cv::Mat& frame, const std::vector<Armor>& armors);

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
};
