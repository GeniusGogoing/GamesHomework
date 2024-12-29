#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2d> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
        cv::Mat* window = (cv::Mat*)userdata;
        window->setTo(0);
    }     
}

void naive_bezier(const std::vector<cv::Point2d> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2d recursive_bezier(const std::vector<cv::Point2d> &control_points, float t,int n,int tail) 
{
    // TODO: Implement de Casteljau's algorithm
    if (n <= 1) return (1 - t) * control_points[tail] + t * control_points[tail - 1];
    return (1 - t) * recursive_bezier(control_points, t, n - 1, tail) + t * recursive_bezier(control_points, t, n - 1, tail - 1);
}

void bezier(const std::vector<cv::Point2d> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    if (control_points.size() <= 1) return;
    for (double t = 0.0; t <= 1.0; t += 0.0001)
    {
        // 计算点的位置
        auto point = recursive_bezier(control_points,t,control_points.size()-1,control_points.size()-1);

        // 反走样
        double y = std::floor(point.y) + 0.5;
        double x = std::floor(point.x) + 0.5;
        for (double i = -1; i <= 1; i++) {
            for (double j = -1; j <= 1; j++) {
                if (y + j < window.rows && y + j >= 0 && x + i < window.cols && x + i >= 0) {
                    cv::Point2d Point2d = {x + i, y + j};
                    cv::Vec2d vec = Point2d - point;
                    double d = sqrtf(vec.dot(vec));
                    window.at<cv::Vec3b>(y + j, x + i)[1] = std::fmax(window.at<cv::Vec3b>(y+j,x+i)[1], 255 - 255 * (d/2.12));
                }
            }
        }
    }
}

// 以下为迭代法计算贝塞尔曲线的算法 
// 原理 贝塞尔曲线上的点的计算在形式上具有规律
/*
  0 1 2 3
0 1
1 1 1
2 1 2 1
3 1 3 3 1
*/
// row 从0开始

std::vector<std::vector<int>> triangle;

void InitTriangle(int row) {
    triangle.clear();

    triangle.push_back({ 1 });
    triangle.push_back({ 1, 1 });
    for (int i = 2; i < row; i++) {
        std::vector<int> row;
        row.push_back(1);
        for (int j = 1; j < i; j++) {
            row.push_back(triangle[i - 1][j - 1] + triangle[i - 1][j]);
        }
        row.push_back(1);
        triangle.push_back(row);
    }
}

int GetFactorA(int row, int col) {
    if (row == col || col == 0) return 1;
    
    return triangle[row - 1][col - 1] + triangle[row - 1][col];
}

void bezierFor(const std::vector<cv::Point2d>& control_points, cv::Mat& window) {
    if (control_points.size() <= 1) return;
    int n = control_points.size();
    InitTriangle(n-1);
    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        // 曲线阶数
        cv::Point2d point = {0, 0};
        for (int i = 0; i < n; i++) {
            point += GetFactorA(n - 1, i) * std::pow((1 - t), n - i - 1) * std::pow(t, i) * control_points[i];
        }

        float y = std::floor(point.y) + 0.5;
        float x = std::floor(point.x) + 0.5;
        for (float i = -1; i <= 1; i++) {
            for (float j = -1; j <= 1; j++) {
                if (y + j < window.rows && y + j >= 0 && x + i < window.cols && x + i >= 0) {
                    cv::Point2d Point2d = { x + i, y + j };
                    cv::Vec2d vec = Point2d - point;
                    float d = sqrtf(vec.dot(vec));
                    window.at<cv::Vec3b>(y + j, x + i)[1] = std::fmax(window.at<cv::Vec3b>(y + j, x + i)[1], 255 - 255 * (d / 2.12));
                }
            }
        }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, &window);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

      /*  if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }*/

        //naive_bezier(control_points, window);
        bezierFor(control_points, window);

        cv::imshow("Bezier Curve", window);

        key = cv::waitKey(20);
    }

return 0;
}
