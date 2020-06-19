// g++ -std=c++11 prediction.cpp -o prediction `pkg-config --cflags --libs opencv`

#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <cmath>

using namespace cv;

int MAX_KERNEL = 10;

cv::Point2f leastSquare(double x[5], cv::Point2f y[5]);
cv::Vec4f expLeastSquare(double x[5], cv::Point2f y[5]);

int main()
{

    cv::Point2f coord_[5];
    double t[5] = {1,2,3,4,5};

    coord_[0] = cv::Point2f(5.436, 2.718);
    coord_[1] = cv::Point2f(14.778, 7.389);
    coord_[2] = cv::Point2f(40.171, 40.171);
    coord_[3] = cv::Point2f(109.196, 54.598);
    coord_[4] = cv::Point2f(296.826, 148.413);

    cv::Point2f eq = leastSquare(t, coord_);
    cv::Vec4i exp_eq = expLeastSquare(t, coord_);

    std::cout << eq.x << " " << eq.y << std::endl;
    std::cout << exp_eq[0] << " " << exp_eq[1] << std::endl;
    std::cout << exp_eq[2] << " " << exp_eq[3] << std::endl;

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
        // std::cout << "Y " << y[i].x << std::endl;

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

cv::Vec4f expLeastSquare(double x[5], cv::Point2f y[5])
{
    double sigma_x = 0;
    double sigma_xx = 0;
    double x_sigma_y = 0;
    double x_sigma_xy = 0;
    double y_sigma_y = 0;
    double y_sigma_xy = 0;

    cv::Vec4f result;

    for (int i = 0; i < 5; i++)
    {

        sigma_x += x[i];
        sigma_xx += (x[i] * x[i]);

        x_sigma_y += std::log(y[i].x);
        x_sigma_xy += (x[i] * std::log(y[i].x));

        y_sigma_y += std::log(y[i].y);
        y_sigma_xy += (x[i] * std::log(y[i].y));
    }

    double ax = (sigma_xx * x_sigma_y) - (sigma_x * x_sigma_xy);
    double Bx = 5 * x_sigma_xy - sigma_x * x_sigma_y;

    double ay = (sigma_xx * y_sigma_y) - (sigma_x * y_sigma_xy);
    double By = 5 * y_sigma_xy - sigma_x * y_sigma_y;

    double delta = 5 * sigma_xx - std::pow(sigma_x, 2);

    double Ax = std::exp((double)ax / delta);
    double Ay = std::exp((double)ay / delta);

    result[0] = Ax;
    result[1] = Bx / delta;
    result[2] = Ay;
    result[3] = By / delta;

    return result;
}