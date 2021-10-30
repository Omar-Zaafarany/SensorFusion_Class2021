#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

using namespace std;

void cornernessHarris()
{
    // load image from file
    cv::Mat img;
    img = cv::imread("../images/img1.png");
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY); // convert to grayscale

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered // Gaussian
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details) k = 0.04 - 0.06.

    // The apertureSize parameter determines the size of the Sobel kernel (3x3, 5x5, etc..). As the size increases, more pixels are part of each convolution process and the edges will get more blurry.

    // The k parameter; with a bigger k, you will get less false corners but you will also miss more real corners (high precision), with a smaller k you will get a lot more corners, so you will miss less true corners, but get a lot of false ones (high recall).

    // source: https://stackoverflow.com/questions/54720646/what-does-ksize-and-k-mean-in-cornerharris/54721585
    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);

    // When the normType is NORM_MINMAX, cv::normalize normalizes _src in such a way that the min value of dst is alpha and max value of dst is beta. cv::normalize does its magic using only scales and shifts (i.e. adding constants and multiplying by constants).
    // source: https://stackoverflow.com/questions/12023958/what-does-cvnormalize-src-dst-0-255-norm-minmax-cv-8uc1/12024043

    // CV_32FC1: 32bits floating point" and C1 for "single channel
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());

    // Convert from 32bit to 8 bit just for visualization
    // try to visualiza both 
    // 32bit >>> white Image
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // visualize results
    string windowName = "Harris Corner Detector Response Matrix";
    cv::namedWindow(windowName, 4);
    cv::imshow(windowName, dst_norm_scaled);
    cv::waitKey(0);

    // TODO: Your task is to locate local maxima in the Harris response matrix 
    // and perform a non-maximum suppression (NMS) in a local neighborhood around 
    // each maximum. The resulting coordinates shall be stored in a list of keypoints 
    // of the type `vector<cv::KeyPoint>`.
    // STUDENTS NEET TO ENTER THIS CODE (C3.2 Atom 4)

    // Look for prominent corners and instantiate keypoints
    vector<cv::KeyPoint> keypoints;

    // cv.KeyPoint(x, y, size[, angle[, response[, octave[, class_id]]]])
    // x: It is the position of the keypoint with respect to the x-axis
    // y: It is the position of the keypoint with respect to the y-axis
    // size: It is the diameter of the keypoint
    // angle: It is the orientation of the keypoint
    // response: Also known as the strength of the keypoint, it is the keypoint detector response on the keypoint
    // octave: It is the pyramid octave in which the keypoint has been detected
    // class_id: It is the object id

    // https://www.pythonpool.com/opencv-keypoint/

    double maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression
    for (size_t j = 0; j < dst_norm.rows; j++)
    {
        for (size_t i = 0; i < dst_norm.cols; i++)
        {

            int response = (int)dst_norm.at<float>(j, i);
            // cout << response << "   "<< dst_norm.at<float>(j, i) << "\n"; 
            // cout << response << "   "<< dst_norm_scaled.at<int>(j, i) << "\n"<< "\n"<< "\n"; 
            // 71   71.7791
            // 71   1212696648

            if (response > minResponse)
            { // only store points above a threshold

                cv::KeyPoint newKeyPoint;
                
                // cout << cv::Point2f(i, j)<< "\n"; //[299, 344]
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response;

                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                bool bOverlap = false;
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                    if (kptOverlap > maxOverlap)
                    {
                        bOverlap = true;
                        if (newKeyPoint.response > (*it).response)
                        {                      // if overlap is >t AND response is higher for new kpt
                            *it = newKeyPoint; // replace old key point with new one
                            break;             // quit loop over keypoints
                        }
                    }
                }
                if (!bOverlap)
                {                                     // only add new key point if no overlap has been found in previous NMS
                    keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list
                }
            }
        } // eof loop over cols
    }     // eof loop over rows

    // visualize keypoints
    windowName = "Harris Corner Detection Results";
    cv::namedWindow(windowName, 5);
    cv::Mat visImage = dst_norm_scaled.clone();
    cv::drawKeypoints(dst_norm_scaled, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow(windowName, visImage);
    cv::waitKey(0);
    // EOF STUDENT CODE
}

int main()
{
    cornernessHarris();
}