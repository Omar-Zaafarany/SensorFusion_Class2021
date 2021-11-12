#include <iostream>
#include <numeric>
#include <fstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>

#include "dataStructures.h"

using namespace std;

void detectObjects2()
{
    // load image from file
    cv::Mat img = cv::imread("../images/img0009.jpg"); //0000000000.png //s_thrun.jpg //img0009.jpg

    // load class names from file
    string yoloBasePath = "../dat/yolo/";
    string yoloClassesFile = yoloBasePath + "coco.names";
    string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    string yoloModelWeights = yoloBasePath + "yolov3.weights"; 

    vector<string> classes;
    ifstream ifs(yoloClassesFile.c_str());
    string line;
    while (getline(ifs, line)) classes.push_back(line);
    
    // load neural network
    // After loading the network, the DNN backend is set to DNN_BACKEND_OPENCV. If OpenCV is built with Intel’s Inference Engine, DNN_BACKEND_INFERENCE_ENGINE should be used instead. The target is set to CPU in the code, as opposed to using DNN_TARGET_OPENCL, which would be the method of choice if a (Intel) GPU was available.
    cv::dnn::Net net = cv::dnn::readNetFromDarknet(yoloModelConfiguration, yoloModelWeights);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // generate 4D blob from input image
    // As data flows through the network, YOLO stores, communicates, and manipulates the information as "blobs": the blob is the standard array and unified memory interface for many frameworks, including Caffe. A blob is a wrapper over the actual data being processed and passed along and also provides synchronization capability between the CPU and the GPU. Mathematically, a blob is an N-dimensional array stored in a C-contiguous fashion. The conventional blob dimensions for batches of image data are number N x channel C x height H x width W. In this nomenclature, N is the batch size of the data. Batch processing achieves better throughput for communication and device processing. For a training batch of 256 images, N would be 256. The parameter C represents the feature dimension, e.g. for RGB images C = 3. In OpenCV, blobs are stored as 4-dimensional cv::Mat array with NCHW dimensions order. More details on blobs can be found here:
    // http://caffe.berkeleyvision.org/tutorial/net_layer_blob.html
    
    // The code below shows how an image loaded from the file is passed through the blobFromImage function to be converted into an input block for the neural network. The pixel values are scaled with a scaling factor of 1/255 to a target range of 0 to 1. It also adjusts the size of the image to the specified size of (416, 416, 416) without cropping.
    cv::Mat blob;
    double scalefactor = 1/255.0;
    cv::Size size = cv::Size(416, 416);
    cv::Scalar mean = cv::Scalar(0,0,0);
    bool swapRB = false; //flag which indicates that swap first and last channels in 3-channel image is necessary.
                         // swap Blue and Red channels (BGR --> RGB)
    bool crop = false; // without cropping
    cv::dnn::blobFromImage(img, blob, scalefactor, size, mean, swapRB, crop);
    // https://www.pyimagesearch.com/2017/11/06/deep-learning-opencvs-blobfromimage-works/#:~:text=I%E2%80%99ve%20provided%20a%20discussion%20of%20each%20parameter%20below%3A
    // https://docs.opencv.org/3.4/d6/d0f/group__dnn.html#:~:text=%E2%97%86%C2%A0-,blobFromImage,-()%20%5B1/2%5D

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////



    // Get names of output layers
    vector<cv::String> names;
    vector<int> outLayers = net.getUnconnectedOutLayers(); // get indices of output layers, i.e. layers with unconnected outputs
    vector<cv::String> layersNames = net.getLayerNames(); // get names of all layers in the network
    
    names.resize(outLayers.size());
    for (size_t i = 0; i < outLayers.size(); ++i) // Get the names of the output layers in names
    {
        names[i] = layersNames[outLayers[i] - 1];
        // cout << "Name = " << names[i] << "\n";
        // Name = yolo_82
        // Name = yolo_94
        // Name = yolo_106
    }

    // invoke forward propagation through network
    vector<cv::Mat> netOutput; 
    net.setInput(blob);
    net.forward(netOutput, names);

    // The result of the forward pass and thus the output of the network is a vector of size C (the number of blob classes) with the first four elements in each class representing the center in x, the center in y as well as the width and height of the associated bounding box. The fifth element represents the trust or confidence that the respective bounding box actually encloses an object. The remaining elements of the matrix are the confidence associated with each of the classes contained in the coco.cfg file. Further on in the code, each box is assigned to the class corresponding to the highest confidence.

    // cout << "netOutput.size() = " << netOutput.size() << "\n"; // 3 --> as if you have three networks


    // Scan through all bounding boxes and keep only the ones with high confidence
    float confThreshold = 0.20;
    vector<int> classIds;
    vector<float> confidences;
    vector<cv::Rect> boxes;

    for (size_t i = 0; i < netOutput.size(); ++i)
    {
        // cout<< "i = "<< i << endl;

        float* data = (float*)netOutput[i].data; 
        // cout << "netOutput[i].data.size() = " << netOutput[i].data.size << endl;
        // data =[normObjCenterx, normObjCentery, normObjWidth, normObjHeight,objectExistenceConfLevel, 
        // confidencelevelC0, confidencelevelC1, .... confidencelevelC79]
       
        // cout<< "netOutput["<< i <<"].rows = " << netOutput[i].rows << " cols = "<< netOutput[i].cols << "\n";
        // netOutput[0].rows = 507 cols = 85  // rows = outputBoxes >> constant parameter for the network
        // netOutput[1].rows = 2028 cols = 85 // cols = (4+1+80) = 85
        // netOutput[2].rows = 8112 cols = 85 // 507 = 13*13*3, 2028 = 13*13*12, 8112 = 13*13*48

        for (int j = 0; j < netOutput[i].rows; ++j, data += netOutput[i].cols) // increment data address by (4+1+80) = 85
        {
            // cout << "data = " << data[0] << " " << data[1] << " " << data[2] << " " << data[3]  << " " << data[4] << " " << data[5] << " " << data[6] << " " << data[7] << " " << data[8]<< "\n"; // similar to below
            // cout << "cols = " << netOutput[i].row(j).colRange(0, netOutput[i].cols) << endl; // similar to above

            cv::Mat scores = netOutput[i].row(j).colRange(5, netOutput[i].cols);
            cv::Point classId;
            double confidence;
            
            // Get the value and location of the maximum score
            cv::minMaxLoc(scores, NULL , &confidence, NULL , &classId); 
            // The function cv::minMaxLoc finds the minimum and maximum element values and their positions with extremes searched across the whole array.
            // NULL is used if not required. 
            // https://cppsecrets.com/users/203110310511410511510410011599115495764103109971051084699111109/C00-OpenCV-cvminmaxLoc.php

            // cout<< i << " " << scores << "\n";
            // cout<< "scores.size() = " << scores.size() << "\n"; //[NoOfClasses x 1] = [80*1]
            // cout<< "confidence = " << confidence << " classId = " << classId << "\n"; //confidence = 0.441967 classId = [2, 0]
            
            if (confidence > confThreshold)
            {
                cv::Rect box; int cx, cy;
                cx = (int)(data[0] * img.cols);
                cy = (int)(data[1] * img.rows);
                box.width = (int)(data[2] * img.cols);
                box.height = (int)(data[3] * img.rows);
                box.x = cx - box.width/2; // left
                box.y = cy - box.height/2; // top
                
                boxes.push_back(box);
                classIds.push_back(classId.x);
                confidences.push_back((float)confidence);
            }
        }

        cout<< endl;

    }

    // perform non-maxima suppression
    float nmsThreshold = 0.4;  // Non-maximum suppression threshold
    vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    std::vector<BoundingBox> bBoxes;
    for (auto it = indices.begin(); it != indices.end(); ++it)
    {
        BoundingBox bBox;
        bBox.roi = boxes[*it];
        bBox.classID = classIds[*it];
        bBox.confidence = confidences[*it];
        bBox.boxID = (int)bBoxes.size(); // zero-based unique identifier for this bounding box
        
        bBoxes.push_back(bBox);
    }
    
    
    // show results
    cv::Mat visImg = img.clone();
    for (auto it = bBoxes.begin(); it != bBoxes.end(); ++it)
    {
        // Draw rectangle displaying the bounding box
        int top, left, width, height;
        top = (*it).roi.y;
        left = (*it).roi.x;
        width = (*it).roi.width;
        height = (*it).roi.height;
        cv::rectangle(visImg, cv::Point(left, top), cv::Point(left + width, top + height), cv::Scalar(0, 255, 0), 2);

        string label = cv::format("%.2f", (*it).confidence);
        label = classes[((*it).classID)] + ":" + label;

        // Display label at the top of the bounding box
        int baseLine;
        cv::Size labelSize = getTextSize(label, cv::FONT_ITALIC, 0.5, 1, &baseLine);
        top = max(top, labelSize.height);
        rectangle(visImg, cv::Point(left, top - round(1.5 * labelSize.height)), cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
        cv::putText(visImg, label, cv::Point(left, top), cv::FONT_ITALIC, 0.75, cv::Scalar(0, 0, 0), 1);
    }

    string windowName = "Object classification";
    cv::namedWindow( windowName, 1 );
    cv::imshow( windowName, visImg );
    cv::waitKey(0); // wait for key to be pressed
}

int main()
{
    detectObjects2();
}