#define CERES_FOUND true
 #include "slam.h"

 using namespace cv;
 using namespace std;
 using namespace cv::sfm;
 //using namespace cv::viz;

 #define MIN_NUM_FEAT 100

 void featureDetection (Mat img_1, vector<Point2f>& points1) {
   vector <KeyPoint> keypoints_1;
   int fast_threshold = 30;
   bool nonmaxSuppression = true;
   FAST (img_1, keypoints_1, fast_threshold, nonmaxSuppression);
   cout << keypoints_1.size() <<"  ";
   Mat out;
   drawKeypoints (img_1, keypoints_1, out, Scalar::all(255));
   imshow ("video", out);
   KeyPoint::convert (keypoints_1, points1, vector<int>());
}
