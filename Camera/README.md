# Sensor Fusion NanoDegree- Camera Course
Camera is the second course in the Sensor Fusion ND. The purpose of this repo is to provide the exercise code to the students, so that they can practice in local system. 

This repo contains lesson-wise exercises and corresponding solutions for Udacity's Sensor Fusion ND. 

## A. List of Lesson-wise Exercises
1. Lesson 2: Autonomous Vehicles and Computer Vision
   - The OpenCV Library
1. Lesson 3: Engineering a Collision Detection System
   - Estimating TTC with Camera
   - Estimating TTC with Lidar
1. Lesson 4: Tracking Image Features
   - Descriptor Matching
   - Gradient-based vs. Binary Descriptors
   - Haris Corner Detection
   - Intensity Gradient and Filtering
   - Overview of Popular Keypoint Detectors
1. Lesson 5: Starter code for "Project: Camera Based 2D Feature Tracking" is available here - https://github.com/udacity/SFND_2D_Feature_Tracking
1. Lesson 6: Combining Camera and Lidar
   - Creating 3D-Objects
   - Lidar-to-Camera Point Projection
   - Object Detection with YOLO
1. Lesson 7: Starter code for "Project: Track an Object in 3D Space" is available here - https://github.com/udacity/SFND_3D_Object_Tracking


## B. Dependencies for Running Locally
1. cmake >= 2.8
    * All OSes: [click here for installation instructions](https://cmake.org/install/)


2. make >= 4.1 (Linux, Mac), 3.81 (Windows)
    * Linux: make is installed by default on most Linux distros
    * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
    * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)


3. OpenCV >= 4.1
    * `Check OpenCV.sh`
    * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
    * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)


4. gcc/g++ >= 5.4 
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)


## C. Build Instructions
1. Fork this repo to your Github account
2. Clone your Github repo.
3. Go to the top level directory for an exercise, and run the following commands on your terminal:
```
mkdir build && cd build
cmake ..
make
./<Executable_File_Name>
```
4. Update back the remote (online) repo so that you can use the updated code in the classroom workspace. 	
