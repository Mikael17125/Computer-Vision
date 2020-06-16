// g++ -std=c++11 prediction.cpp -o prediction `pkg-config --cflags --libs opencv`

#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <cmath>

using namespace cv;

int MAX_KERNEL = 10;

cv::Point2f leastSquare(double x[6], cv::Point2f y[6]);

int main()
{

    cv::Point2f coord_[5];
    double t[5] = {2,4,6,8,10};

    coord_[0] = cv::Point2f(42, 1);
    coord_[1] = cv::Point2f(48.4, 2);
    coord_[2] = cv::Point2f(51.3, 3);
    coord_[3] = cv::Point2f(56.3, 4);
    coord_[4] = cv::Point2f(58.6, 5);

    cv::Point2f eq = leastSquare(t, coord_);

    std::cout << eq.x << " " << eq.y << std::endl;

    return 0;
}

cv::Point2f leastSquare(double x[5], cv::Point2f y[5])
{

    double sigma_x = 0;
    double sigma_xx = 0;
    double x_sigma_y = 0;
    double x_sigma_xy = 0;

    cv::Point2f result;

    // std::cout<<"x "<<x[0]<<std::endl;

    for (int i = 0; i < 5; i++)
    {
        std::cout << "Y " << y[i].x << std::endl;

        sigma_x += x[i];
        sigma_xx += (x[i] * x[i]);
        x_sigma_y += y[i].x;
        x_sigma_xy += (x[i] * y[i].x);
    }


    double Ax = (sigma_xx * x_sigma_y) - (sigma_x * x_sigma_xy);
    double Bx = 5 * x_sigma_xy - sigma_x * x_sigma_y;
    double delta = 5 * sigma_xx - std::pow(sigma_x, 2);

    std::cout<<"Sigma X  "<<sigma_x<<std::endl;
    std::cout<<"Sigma Y  "<<x_sigma_y<<std::endl;
    std::cout<<"Sigma XY "<<x_sigma_xy<<std::endl;
    std::cout<<"Sigma XX "<<sigma_xx<<std::endl;

    result.x = Ax / delta;
    result.y = Bx / delta;

    return result;
}