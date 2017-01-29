#define CERES_FOUND true
#include <opencv2/core.hpp>
#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>
#include<cvaux.h>
#include<highgui.h>
#include<stdio.h>
#include<stdlib.h>
#include<cxcore.h>
#include<cv.h>
#include<opencv2/imgproc.hpp>
#include<opencv2/core/core_c.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

/*#include <iostream>
#include <fstream>
#include <string>
//#include<vo_features.h>
//#include<opencv2/calib3d/calib3d_c.h>

using namespace cv;
using namespace std;
using namespace cv::sfm;

void getReprojectionError(Mat R_PrevFRame, Mat t_PrevFrame, Mat K, Mat R_CurrFrame, Mat t_CurrFrame, vector<Point2f> prevFeatures, vector<Point2f> currFeatures, Mat reprojectionError){

    vector <Point2f> prevFrame = prevFeatures;
    vector<Point2f> currFrame = currFeatures;
    vector<vector<Point2f> > pointSet2D;
    pointSet2D.push_back(prevFrame);
    pointSet2D.push_back(currFrame);
    Mat R_prevFrame = R_PrevFRame;
    Mat t_prevFrame = t_PrevFrame;
    R_prevFrame.push_back(t_prevFrame);
    Mat R_currFrame = R_CurrFrame;
    Mat t_currFrame = t_CurrFrame;
    R_currFrame.push_back(t_currFrame);
    vector<Mat> projectionMatrices;
    projectionMatrices.push_back(R_prevFrame);
    projectionMatrices.push_back(R_currFrame);
    Mat points3d;

    triangulatePoints(pointSet2D,projectionMatrices,points3d);
    //Mat ones = Mat::ones(1,points3d.cols,CV_64F);
    //points3d.push_back(ones);
    //vector<Point2f> reprojectionError;

    for(int i=0;i<points3d.cols;i++){

        Mat pt_ImageEuclidean;
        Mat pt_ImageHomogenous;
        Mat pt_WorldHomogenous;
        Mat pt_WorldEuclidean = points3d.col(i);
        euclideanToHomogeneous(pt_WorldEuclidean,pt_WorldHomogenous);
        pt_ImageHomogenous = K*R_currFrame*pt_WorldEuclidean;
        homogeneousToEuclidean(pt_ImageHomogenous,pt_ImageEuclidean);
        reprojectionError.col(i) = currFrame.at(i) - pt_ImageEuclidean;

    }
}*/
