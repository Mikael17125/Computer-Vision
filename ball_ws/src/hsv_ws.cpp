#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

int MAX_KERNEL = 10;

int main()
{
    cv::namedWindow("Track");
    cv::namedWindow("Output");
    int H_low = 0, S_low = 24, V_low = 46;
    int H_high = 24, S_high = 255, V_high = 255;

    cv::Mat img = cv::imread("ball.jpg");
    cv::Mat filt(img.size(), img.type());

    for (int i = 1; i < MAX_KERNEL; i = i + 2)
    {
        cv::blur(img, filt, cv::Size(i, i), cv::Point(-1, -1));
    }

    cv::Mat img_hsv;

    cv::Mat out(img.size(), img.type());
    cv::Mat mask(img.size(), img.type());

    cv::cvtColor(filt, img_hsv, cv::COLOR_BGR2HSV);

    std::cout << "IMG: " << img.cols << " MASK: " << mask.cols << " OUT: " << out.cols << std::endl;
    std::cout << "IMG: " << img.rows << " MASK: " << mask.rows << " OUT: " << out.rows << std::endl;
    while (true)
    {
        cv::createTrackbar("H_LOW", "Track", &H_low, 255);
        cv::createTrackbar("S_LOW", "Track", &S_low, 255);
        cv::createTrackbar("V_LOW", "Track", &V_low, 255);
        cv::createTrackbar("H_HIGH", "Track", &H_high, 255);
        cv::createTrackbar("S_HIGH", "Track", &S_high, 255);
        cv::createTrackbar("V_HIGH", "Track", &V_high, 255);
        cv::inRange(img_hsv, cv::Scalar(H_low, S_low, V_low), cv::Scalar(H_high, S_high, V_high), mask);

        cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);

        cv::bitwise_and(img, mask, out);

        cv::imshow("Output", out);
        cv::imshow("Track", filt);
        cv::waitKey(1);
    }

    return 1;
}
