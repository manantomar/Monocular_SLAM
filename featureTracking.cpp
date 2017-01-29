#define CERES_FOUND true
 #include "slam.h"

 using namespace cv;
 using namespace std;
 using namespace cv::sfm;
 //using namespace cv::viz;

 #define MIN_NUM_FEAT 100

 void featureTracking (Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status) {
   vector <KeyPoint> keypoints_1;
   vector <float> err;
   Size winSize = Size(21, 21);
   TermCriteria termcrit = TermCriteria (TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);
   calcOpticalFlowPyrLK (img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);
   cout << points2.size() << " bing   ";
   cout << points1.size() << " dob   ";
   vector <KeyPoint> tracked_points;
   for (vector<Point2f>::const_iterator it = points2.begin();
		                            it != points2.end(); it++) {
      cv::KeyPoint kp(*it, 8);
      tracked_points.push_back(kp);
   }
   Mat out;
   drawKeypoints (img_1, tracked_points, out, Scalar::all(255));
   imshow ("tracking", out);
   int indexCorrection = 0;
   for (int i = 0; i<status.size(); i++) {
     Point2f pt = points2.at(i - indexCorrection);
     if ((status.at(i) == 0) || (pt.x<0) || (pt.y<0)) {

       if ((pt.x<0) || (pt.y<0)) {
         status.at(i) = 0;
       }
       points1.erase(points1.begin() + (i - indexCorrection));
       points2.erase(points2.begin() + (i - indexCorrection));
       indexCorrection++;
     }
   }
   cout << "\nsize of feature matrix" << points1.size() << "\n";
 }
