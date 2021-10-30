#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

using namespace std;

void detKeypoints1()
{
    // load image from file and convert to grayscale
    cv::Mat imgGray;
    cv::Mat img = cv::imread("../images/img1.png");
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

    // Shi-Tomasi detector
    int blockSize = 6;       //  size of a block for computing a derivative covariation matrix over each pixel neighborhood as the case in any filter
    
    double maxOverlap = 0.0; // max. permissible overlap between two features in % 
    // if there are multiple overlapped keypoints only one is selected using NMS

    double minDistance = (1.0 - maxOverlap) * blockSize; // minDistance = blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints (corners)
    // max(1.0, minDistance) to make use not to divide by zero

    double qualityLevel = 0.01; // minimal accepted quality of image corners similar to minResponse in harris
    double k = 0.04;
    bool useHarris = false;

    vector<cv::KeyPoint> kptsShiTomasi;
    vector<cv::Point2f> corners; //point2f : 2D datatype of type float that holds the point coordinates
    
    double t = (double)cv::getTickCount();
    cv::goodFeaturesToTrack(imgGray, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, useHarris, k);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi with n= " << corners.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    for (auto it = corners.begin(); it != corners.end(); ++it)
    { // add corners to result vector

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        kptsShiTomasi.push_back(newKeyPoint);
    }

    // visualize results
    
    // drawKeypoints() 
    //https://docs.opencv.org/3.4.15/d4/d5d/group__features2d__draw.html#gab958f8900dd10f14316521c149a60433
    // Parameters
    // image	Source image.
    // keypoints	Keypoints from the source image.
    // outImage	Output image. Its content depends on the flags value defining what is drawn in the output image. See possible flags bit values below.
    // color	Color of keypoints.
    // flags	Flags setting drawing features. Possible flags bit values are defined by DrawMatchesFlags. See details above in drawMatches .

    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, kptsShiTomasi, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = "Shi-Tomasi Results";
    cv::namedWindow(windowName, 1);
    imshow(windowName, visImage);

    // TODO: use the OpenCV library to add the FAST detector
    // in addition to the already implemented Shi-Tomasi 
    // detector and compare both algorithms with regard to 
    // (a) number of keypoints, (b) distribution of 
    // keypoints over the image and (c) processing speed.
    // STUDENT CODE
    int threshold = 30;   // difference between intensity of the central pixel and pixels of a circle around this pixel
    bool bNMS = true;     // perform non-maxima suppression on keypoints
    cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16; // TYPE_9_16, TYPE_7_12, TYPE_5_8
    
    cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create(threshold, bNMS, type);

    vector<cv::KeyPoint> kptsFAST;
    t = (double)cv::getTickCount();
    detector->detect(imgGray, kptsFAST);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "FAST with n= " << kptsFAST.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    visImage = img.clone();
    cv::drawKeypoints(img, kptsFAST, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    windowName = "FAST Results";
    cv::namedWindow(windowName, 2);
    imshow(windowName, visImage);
    cv::waitKey(0);

    // EOF STUDENT CODE
}

int main()
{
    detKeypoints1();
}