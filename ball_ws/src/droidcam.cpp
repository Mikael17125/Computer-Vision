// g++ -std=c++11 droidcam.cpp -o droidcam `pkg-config --cflags --libs opencv`

//Taylor J.R Introduction to error analysis 2ed.pdf

#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <cmath>

using namespace cv;

int MAX_KERNEL = 10;

cv::Vec4f leastSquare(double x[6], cv::Point y[6]);
cv::Vec4f expLeastSquare(double x[6], cv::Point y[6]);

int main()
{
    auto start = std::chrono::system_clock::now();

    cv::Mat out;
    cv::Mat element5(5, 5, CV_8U, cv::Scalar(1));
    cv::Mat closed, opened, dilated, eroded;
    std::vector<cv::Vec3f> detected;
    cv::namedWindow("Track");

    cv::Point data[6];
    cv::Vec4i pred;

    double time[6] = {0, 0, 0, 0, 0, 0};

    int radius = 0;

    data[0] = cv::Point(0, 0);
    data[1] = cv::Point(0, 0);
    data[2] = cv::Point(0, 0);
    data[3] = cv::Point(0, 0);
    data[4] = cv::Point(0, 0);
    data[5] = cv::Point(0, 0);

    // // Malam
    // int H_low = 117, S_low = 49, V_low = 82;
    // int H_high = 255, S_high = 185, V_high = 255;
    // int DP = 13;
    // //Siang
    // int H_low = 107, S_low = 70, V_low = 165;
    // int H_high = 255, S_high = 255, V_high = 255;
    // int DP = 10;

    // //White
    int H_low = 0, S_low = 0, V_low = 250;
    int H_high = 255, S_high = 255, V_high = 255;
    int DP = 10;

    VideoCapture cap("http://192.168.100.5:4747/mjpegfeed");

    if (!cap.isOpened())
        return 0;
    while (true)
    {
        Mat frame, hsv_out;
        cap >> frame;
        if (frame.empty())
            break; // end of video stream
        cv::Mat mask(frame.size(), frame.type());
        cv::Mat filt(frame.size(), frame.type());

        for (int i = 1; i < MAX_KERNEL; i = i + 2)
        {
            cv::blur(frame, filt, cv::Size(i, i), cv::Point(-1, -1));
        }

        cv::cvtColor(filt, hsv_out, COLOR_BGR2HSV);
        cv::cvtColor(filt, out, COLOR_BGR2GRAY);

        cv::createTrackbar("H_LOW", "Track", &H_low, 255);
        cv::createTrackbar("S_LOW", "Track", &S_low, 255);
        cv::createTrackbar("V_LOW", "Track", &V_low, 255);
        cv::createTrackbar("H_HIGH", "Track", &H_high, 255);
        cv::createTrackbar("S_HIGH", "Track", &S_high, 255);
        cv::createTrackbar("V_HIGH", "Track", &V_high, 255);
        cv::createTrackbar("DP", "Track", &DP, 15);

        cv::inRange(hsv_out, cv::Scalar(H_low, S_low, V_low), cv::Scalar(H_high, S_high, V_high), mask);

        //Fill Hole (dilation followed by erotion)
        // cv::morphologyEx(mask, closed, cv::MORPH_CLOSE, element5);
        //Remove Noise (erotion followed by dilation)
        cv::morphologyEx(mask, opened, cv::MORPH_OPEN, element5);

        //Assume mask as Gray
        // cv::cvtColor(opened, opened, cv::COLOR_GRAY2BGR);
        // cv::bitwise_and(frame, opened, out);

        auto now = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_scnd = now - start;

        cv::GaussianBlur(opened, opened, Size(9, 9), 2, 2);
        cv::HoughCircles(opened, detected, CV_HOUGH_GRADIENT, DP, 50, 200, 100, 0, 0);

        // std::cout << "Elapsed time " << elapsed_scnd.count() << std::endl;
        // std::cout << "Detected Size " << detected.size() << std::endl;
        // std::cout << "============== " << std::endl;

        for (size_t i = 0; i < detected.size(); i++)
        {
            Point center(cvRound(detected[i][0]), cvRound(detected[i][1]));
            radius = cvRound(detected[i][2]);

            if (radius < 70)
            {
                cv::circle(frame, center, radius, Scalar(0, 0, 255), 3, 8, 0);
                // std::cout <<"Radius : "<< radius << std::endl;
            }
        }

        if (radius != 0)
        {
            for (int i = 5; i >= 0; i--)
            {
                // cv::circle(frame, data[i], 6, cv::Scalar(i * 10, i * 20, i * 30), CV_FILLED, 8, 0);

                if (i != 0)
                {
                    data[i] = data[i - 1];
                    time[i] = time[i - 1];
                }
            }

            time[0] = elapsed_scnd.count();
            data[0] = cv::Point(detected[0][0], detected[0][1]);

            pred = leastSquare(time, data);
        }
        else
        {
            pred[0] = 0;
            pred[1] = 0;
            pred[2] = 0;
            pred[3] = 0;
        }

        for (int i = 0; i < 5; i++)
        {
            // A + Bx

            int t_pred = elapsed_scnd.count() + i;
            int x = pred[0] + pred[1] * t_pred;
            int y = pred[2] + pred[3] * t_pred;

            cv::Point2i center(x, y);

            cv::circle(frame, center, 6, cv::Scalar(i * 10, i * 20, i * 30), CV_FILLED, 8, 0);
        }

        imshow("MASK", opened);
        // imshow("DILATED", dilated);
        cv::imshow("OUTPUT", frame);
        // imshow("HSV", hsv_out);

        if (waitKey(10) == 27)
            break; // stop capturing by pressing ESC
    }

    return 0;
}

cv::Vec4f leastSquare(double x[6], cv::Point y[6])
{

    double sigma_x = 0;
    double sigma_xx = 0;
    double x_sigma_y = 0;
    double x_sigma_xy = 0;
    double y_sigma_y = 0;
    double y_sigma_xy = 0;

    cv::Vec4f result;

    for (int i = 0; i < 6; i++)
    {

        sigma_x += x[i];
        sigma_xx += (x[i] * x[i]);

        x_sigma_y += y[i].x;
        x_sigma_xy += (x[i] * y[i].x);

        y_sigma_y += y[i].y;
        y_sigma_xy += (x[i] * y[i].y);
    }

    double Ax = (sigma_xx * x_sigma_y) - (sigma_x * x_sigma_xy);
    double Bx = 6 * x_sigma_xy - sigma_x * x_sigma_y;

    double Ay = (sigma_xx * y_sigma_y) - (sigma_x * y_sigma_xy);
    double By = 6 * y_sigma_xy - sigma_x * y_sigma_y;

    double delta = 6 * sigma_xx - std::pow(sigma_x, 2);

    // std::cout << "Sigma X  " << sigma_x << std::endl;
    // std::cout << "Sigma Y  " << x_sigma_y << std::endl;
    // std::cout << "Sigma XY " << x_sigma_xy << std::endl;
    // std::cout << "Sigma XX " << sigma_xx << std::endl;

    result[0] = Ax / delta;
    result[1] = Bx / delta;
    result[2] = Ay / delta;
    result[3] = By / delta;

    return result;
}

cv::Vec4f expLeastSquare(double x[6], cv::Point y[6])
{
    double sigma_x = 0;
    double sigma_xx = 0;
    double x_sigma_y = 0;
    double x_sigma_xy = 0;
    double y_sigma_y = 0;
    double y_sigma_xy = 0;

    cv::Vec4f result;

    for (int i = 0; i < 6; i++)
    {

        sigma_x += x[i];
        sigma_xx += (x[i] * x[i]);

        x_sigma_y += std::log(y[i].x);
        x_sigma_xy += (x[i] * std::log(y[i].x));

        y_sigma_y += std::log(y[i].y);
        y_sigma_xy += (x[i] * std::log(y[i].y));
    }

    double ax = (sigma_xx * x_sigma_y) - (sigma_x * x_sigma_xy);
    double Bx = 6 * x_sigma_xy - sigma_x * x_sigma_y;

    double ay = (sigma_xx * y_sigma_y) - (sigma_x * y_sigma_xy);
    double By = 6 * y_sigma_xy - sigma_x * y_sigma_y;

    double delta = 6 * sigma_xx - std::pow(sigma_x, 2);

    double Ax = std::exp((double)ax / delta);
    double Ay = std::exp((double)ay / delta);

    result[0] = Ax;
    result[1] = Bx / delta;
    result[2] = Ay;
    result[3] = By / delta;

    return result;
}