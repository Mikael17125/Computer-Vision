#include "opencv2/opencv.hpp"
using namespace cv;

int MAX_KERNEL = 10;

int main(int argc, char **argv)
{
    cv::Mat out;

    cv::namedWindow("Track");
    int H_low = 147, S_low = 31, V_low = 0;
    int H_high = 255, S_high = 192, V_high = 255;

    VideoCapture cap("http://192.168.100.5:4747/mjpegfeed?480x320");
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

        cvtColor(filt, hsv_out, COLOR_BGR2HSV);
        cv::createTrackbar("H_LOW", "Track", &H_low, 255);
        cv::createTrackbar("S_LOW", "Track", &S_low, 255);
        cv::createTrackbar("V_LOW", "Track", &V_low, 255);
        cv::createTrackbar("H_HIGH", "Track", &H_high, 255);
        cv::createTrackbar("S_HIGH", "Track", &S_high, 255);
        cv::createTrackbar("V_HIGH", "Track", &V_high, 255);
        cv::inRange(hsv_out, cv::Scalar(H_low, S_low, V_low), cv::Scalar(H_high, S_high, V_high), mask);
        cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
        cv::bitwise_and(frame, mask, out);

        imshow("ORIGINAL", frame);
        cv::imshow("Output", out);
        imshow("HSV", hsv_out);
        if (waitKey(10) == 27)
            break; // stop capturing by pressing ESC
    }
    // the camera will be closed automatically upon exit
    // cap.close();
    return 0;
}