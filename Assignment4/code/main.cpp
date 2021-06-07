#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}
void antialiasing(cv::Point2f point,  cv::Mat &window, int rgbInt)
{
  cv::Point2f point_l(std::floor(point.y) - 1, std::floor(point.x));
        cv::Point2f point_r(std::floor(point.y) + 1, std::floor(point.x));
        cv::Point2f point_t(std::floor(point.y), std::floor(point.x) + 1);
        cv::Point2f point_b(std::floor(point.y) - 1, std::floor(point.x) - 1);

        float delta_x = (point.x - std::floor(point.x));
        float delta_y = (point.y - std::floor(point.y));

        float deviation = sqrt(delta_x * delta_x + delta_y * delta_y);
       
        std::vector<cv::Point2f> vecList;
        vecList.push_back(point_l);
        vecList.push_back(point_r);
        vecList.push_back(point_t);
        vecList.push_back(point_b);

        for (auto vec:vecList)
        {   
            float distance = sqrt(pow((vec.x - point.x), 2) - pow((vec.y - point.y), 2));
            int color = window.at<cv::Vec3b>(vec.x, vec.y)[rgbInt];
            window.at<cv::Vec3b>(vec.x, vec.y)[rgbInt] = std::max(color * 1.0f, deviation / distance * 255);
        }
}
void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
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

        antialiasing(point, window, 2);
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if (points.size() == 1) {
        return points[0];
    }

    // std::cout<<"points.size():"<<points.size()<<std::endl;

    std::vector<cv::Point2f> new_points;
    for (int idx = 0; idx < points.size() - 1; ++idx)
    {  
        auto left = points[idx];
        auto right = points[idx+1];
        auto interpolate = left + t * (right - left);
        new_points.push_back(interpolate);
    }
    
    return recursive_bezier(new_points, t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
 for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        cv::Point2f point =  recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
        antialiasing(point,window, 1);
    
    }
}



int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);
    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
