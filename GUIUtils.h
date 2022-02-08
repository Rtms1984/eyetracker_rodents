#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ctime>
#include <string>
#include <sstream>


cv::Rect drawString(cv::Mat img, std::string text, cv::Point coord, cv::Scalar color, float fontScale = 0.6f, int thickness = 1, int fontFace = cv::FONT_HERSHEY_COMPLEX);
cv::Rect drawButton(cv::Mat img, std::string text, cv::Point coord, int minWidth = 0);
void onMouse(int event, int x, int y, int, void*);
void on_trackbarSensibility(int, void*);

std::string addTimeDateToString(std::string &prefix, std::string &suffix);
int ReadAvi(std::string filename);
//template <typename T> T mediana(std::vector<T> &v, float percentile);


