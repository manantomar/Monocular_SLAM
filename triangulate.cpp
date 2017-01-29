#define CERES_FOUND true
 #include "slam.h"

 using namespace cv;
 using namespace std;
 using namespace cv::sfm;


void triangulateLandmarks (vector <Point2f> prev_features, vector <Point2f> curr_features, Mat prev_R,
                                                            Mat prev_t, Mat Rs, Mat ts, Mat points3d) {
   vector <Point2f> prev = prev_features;
   vector <Point2f> curr = curr_features;
   vector <Mat> pointSet2D;
   vector <Mat> projectionMatrices;
   Mat P_prevFrame = Mat::zeros (3,4,CV_64F);
   Mat M = Mat::zeros (2,curr.size(),CV_64F);
   for (int i=0;i<curr.size();i++) {
     M.at<double>(0,i) = curr.at(i).x;
     M.at<double>(1,i) = curr.at(i).y;
   }
   Mat N = Mat::zeros (2,prev.size(),CV_64F);
   for (int i=0;i<prev.size();i++) {
     N.at <double> (0,i) = prev.at(i).x;
     N.at <double> (1,i) = prev.at(i).y;
   }
   pointSet2D.push_back(N);
   pointSet2D.push_back(M);
   for (int i=0;i<2;i++) {
     cout << "\n";
   for (int j=0;j<4;j++) {
     cout<<M.at<double>(i,j)<<"\t";
   }
   }
   for (int j=0;j<4;j++) {
     cout<<curr.at(j)<<"\t";
   }
   prev_R.copyTo (P_prevFrame(cv::Rect_ <double> (0,0,3,3)));
   prev_t.copyTo (P_prevFrame(cv::Rect_ <double> (3,0,1,3)));
   cout << "\ndebugging triangulation\n";
   Mat P_currFrame = Mat::zeros (3,4,CV_64F);
   Rs.copyTo (P_currFrame(cv::Rect_ <double> (0,0,3,3)));
   ts.copyTo (P_currFrame(cv::Rect_ <double> (3,0,1,3)));
   projectionMatrices.push_back(P_prevFrame);
   projectionMatrices.push_back(P_currFrame);
   cout << "\nprinting 3d point matrix size\t" << projectionMatrices.size() << "\n";
   cout << "\ndebugging triangulation\n";
   triangulatePoints (pointSet2D,projectionMatrices,points3d);

 }
