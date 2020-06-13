// g++ droidcam.cpp -o droidcam `pkg-config --cflags --libs opencv`

#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <iostream>

using namespace cv;

int MAX_KERNEL = 10;

// class Linked_List{

//     Linked_List *Next;
//     int data;

// };

int main(int argc, char **argv)
{
    cv::Mat out;
    cv::Mat element5(5, 5, CV_8U, cv::Scalar(1));
    cv::Mat closed, opened, dilated, eroded;
    std::vector<cv::Vec3f> detected;
    cv::namedWindow("Track");

    cv::Point pred[6];

    pred[0] = cv::Point(0, 0);
    pred[1] = cv::Point(0, 0);
    pred[2] = cv::Point(0, 0);
    pred[3] = cv::Point(0, 0);
    pred[4] = cv::Point(0, 0);
    pred[5] = cv::Point(0, 0);

    // // Malam
    // int H_low = 126, S_low = 40, V_low = 157;
    // int H_high = 255, S_high = 192, V_high = 255;
    // // DP = 2.5
    //Siang
    int H_low = 139,
        S_low = 40, V_low = 155;
    int H_high = 255, S_high = 156, V_high = 255;
    // DP = 5

    VideoCapture cap("http://192.168.100.5:4747/mjpegfeed");
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.

    if (!cap.isOpened())
        return 0;
    for (;;)
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

        cv::inRange(hsv_out, cv::Scalar(H_low, S_low, V_low), cv::Scalar(H_high, S_high, V_high), mask);

        //Fill Hole (dilation followed by erotion)
        // cv::morphologyEx(mask, closed, cv::MORPH_CLOSE, element5);
        //Remove Noise (erotion followed by dilation)
        cv::morphologyEx(mask, opened, cv::MORPH_OPEN, element5);

        //Assume mask as Gray
        // cv::cvtColor(opened, opened, cv::COLOR_GRAY2BGR);
        // cv::bitwise_and(frame, opened, out);

        cv::GaussianBlur(opened, opened, Size(9, 9), 2, 2);

        cv::HoughCircles(opened, detected, CV_HOUGH_GRADIENT, 5, 50, 200, 100, 0, 0);

        // // for (size_t i = 0; i < detected.size(); i++)
        // // {
        cv::Point center(cvRound(detected[0][0]), cvRound(detected[0][1]));
        int radius = cvRound(detected[0][2]);

        cv::circle(frame, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        cv::circle(frame, center, radius, cv::Scalar(0, 255, 0), 3, 8, 0);
        // // }

        // std::cout << "X : " << detected[0][0] << " Y : " << detected[0][1] << std::endl;

        for (int i = 5; i > 0 ; i--)
        {
           
            pred[i] = pred[i-1];
        
        }

        pred[0] = cv::Point(detected[0][0], detected[0][1]);

        for (int i = 0; i < 6; i++)
        {
            cv::circle(frame, pred[i], 6, cv::Scalar(0, i*10, i * 30), CV_FILLED, 8, 0);

            // std::cout << "pred : " << pred[i]  << std::endl;
        }

        // imshow("ORIGINAL", frame);
        imshow("MASK", opened);
        // imshow("DILATED", dilated);
        cv::imshow("OUTPUT", frame);
        // imshow("HSV", hsv_out);
        if (waitKey(10) == 27)
            break; // stop capturing by pressing ESC
    }
    // the camera will be closed automatically upon exit
    // cap.close();
    return 0;
}