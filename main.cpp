 #define CERES_FOUND true
 #include "slam.h"

 using namespace cv;
 using namespace std;
 using namespace cv::sfm;
 //using namespace cv::viz;

 #define MIN_NUM_FEAT 100

 void featureDetection (Mat img_1, vector<Point2f>& points1);

void featureTracking (Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status);

 bool keyFrameSelection(Mat Cp, bool keyframe_change) {
   double Cp_magnitude;
   for (int i=0;i<Cp.rows;i++) {
     Cp_magnitude += pow((Cp.at <double>(i,0)),2);
   }
   Cp_magnitude = pow(Cp_magnitude,0.5);
   if (keyframe_change == true || Cp_magnitude > 20.0) {
     Cp = Mat::zeros (3,1,CV_64F);
     return true;
   }
   else {
     return false;
   }
 }

 void triangulateLandmarks (vector <Point2f> prev_features, vector <Point2f> curr_features, Mat prev_R,
                                                            Mat prev_t, Mat Rs, Mat ts, Mat points3d);

 int main (int argc, char **argv) {
   Mat prevFrame;
   Mat currFrame;
   Mat prevFrame_c;
   Mat currFrame_c;
   Mat R_f, t_f;
   vector <Mat> points3d_set;
   vector <Point2f> prev_features;
   vector <Point2f> curr_features;
   vector <Point2f> points1;
   double focal = 132.07;
   cv::Point2d pp (169.6, 95.25);
   int x;
   int y;
    Matx33d Kc = Matx33d( 132.07, 0, 169.6,
                       0, 132.07, 95.25,
                       0, 0,  1);


   cvNamedWindow ("video", CV_WINDOW_AUTOSIZE);
   cvNamedWindow ("tracking", CV_WINDOW_AUTOSIZE);
   cvNamedWindow ("reproject", CV_WINDOW_AUTOSIZE);
   namedWindow( "Trajectory", WINDOW_AUTOSIZE );
   //cvNamedWindow ("visualize", CV_WINDOW_AUTOSIZE);
   VideoCapture cap(0);
   if (!cap.isOpened()) {
		return -1;
   }
   Mat img_1_c;
   Mat img_1;
   cap >> img_1_c;
   cvtColor (img_1_c, img_1, COLOR_BGR2GRAY);
   featureDetection (img_1, points1);
   prevFrame = img_1;
   prev_features = points1;
   std::vector <Mat> t ;
   for (int j=0;j<t.size();j++) {
     t[j].push_back(Mat(1, 1, CV_64F, Scalar::all(0)));
   }
   vector <Mat> R ;
   Mat Rs = Mat::ones (3,3,CV_64F);
   Mat ts = Mat::zeros (3,1,CV_64F);
   Mat Cp = Mat::zeros (3,1,CV_64F);
   Mat Cp_temp = Mat::zeros (3,1,CV_64F);
   Mat prev_R = Mat::ones (3,3,CV_64F);
   Mat prev_t = Mat::zeros (3,3,CV_64F);
   Mat K = (Mat_ <double> (3,3) << 132.07, 0.0, 0.0, 0.0, 132.07, 0.0, 169.6, 95.25, 1.0);
   Mat temp_t = Mat::zeros (3,1,CV_64F);
   Mat temp_R = Mat::ones (3,3,CV_64F);
   vector <vector <Point2f> > Points2d;
   int n_views = 0;
   int index_right_sol;

    viz::Viz3d window("Coordinate Frame");
             window.setWindowSize(Size(500,500));
             window.setWindowPosition(Point(150,150));
             window.setBackgroundColor(); // black by default
   vector<Affine3d> path;
   vector<Vec3d> point_cloud_est;
   Mat rot_vec = Mat::zeros(1,3,CV_32F);
   Mat rot_mat;
   Rodrigues(rot_vec, rot_mat);
   Mat traj = Mat::zeros(600, 600, CV_8UC3);
   bool keyframe_change = false;

   while (1<2) {
     n_views += 1;
     cap >> currFrame_c;
     cvtColor (currFrame_c, currFrame, COLOR_BGR2GRAY);
     vector <uchar> status;
     keyframe_change = false;
     //featureDetection(prevFrame, prev_features);
     featureTracking (prevFrame, currFrame, prev_features, curr_features, status);
     if (prev_features.size() < MIN_NUM_FEAT) {
       keyframe_change = true;

     }
    /*Mat T;
     Mat T2;
      Mat test = Mat::zeros (2,prev_features.size(),CV_64F);
   for (int i=0;i<prev_features.size();i++) {
     test.at <double> (0,i) = prev_features.at(i).x;
     test.at <double> (1,i) = prev_features.at(i).y;
   }
   Mat test2 = Mat::zeros (2,curr_features.size(),CV_64F);
   for (int i=0;i<curr_features.size();i++) {
     test2.at <double> (0,i) = curr_features.at(i).x;
     test2.at <double> (1,i) = curr_features.at(i).y;
   }
     vector <Point2f> normalized_prev_features;
     vector <Point2f> normalized_curr_features;
     //cout << "debugging normalization";
     normalizeIsotropicPoints(Mat(prev_features),normalized_prev_features, T );
     normalizeIsotropicPoints(Mat(curr_features),normalized_curr_features, T2 );
     for (int i=0;i<normalized_prev_features.rows;i++) {
     cout << "\n";
     for (int j=0;j<normalized_prev_features.cols;j++) {
     cout << "\t" << normalized_prev_features.at<double>(i,j);
     }
     }
      cout << "debugging normalization";
      cout << "size" << normalized_prev_features.size();
      cout << "size" << prev_features.size();
     /*vector <Point2f> normalized_test = prev_features;
     for (int i=0;i<normalized_prev_features.rows;i++) {
     normalized_test.at(i).x = normalized_prev_features.at <double> (i,0);
     normalized_test.at(i).y = normalized_prev_features.at <double> (i,1);
   }
   vector <Point2f> normalized_test2 = curr_features;
     for (int i=0;i<normalized_curr_features.rows;i++) {
     normalized_test2.at(i).x = normalized_curr_features.at <double> (i,0);
     normalized_test2.at(i).y = normalized_curr_features.at <double> (i,1);
   }
     cout << "size" << normalized_prev_features.size();*/
     vector <Point2f> prev_features_n;
     vector <Point2f> curr_features_n;
     cv::normalize(prev_features,prev_features_n,1.414,0,NORM_L2,-1);
     cv::normalize(curr_features,curr_features_n,1.414,0,NORM_L2,-1);

     Mat F = findFundamentalMat(prev_features_n,curr_features_n,FM_RANSAC);
     Mat x1 = Mat::zeros (2,1,CV_64F);
     Mat x2 = Mat::zeros (2,1,CV_64F);
     x1.at <double> (0,0) = prev_features.at(0).x;
     x1.at <double> (1,0) = prev_features.at(0).y;
     x2.at <double> (0,0) = curr_features.at(0).x;
     x2.at <double> (1,0) = curr_features.at(0).y;
     printf("%f\n",x2.at <double> (0,0));
     printf("%f",curr_features.at(0).x);
     double data[] = {132.07 , 0.0, 0.0,
                      0.0, 132.07, 0.0,
                      169.6, 95.25, 1.0};
     if (F.rows == 3 && F.cols == 3) {
       Mat E = Mat (3,3, CV_64F);
       if (F.rows != K.rows) {
         printf("\n no - rows");
       }
       if (F.cols != K.cols) {
         cout << "\n no -cols";
       }
     essentialFromFundamental (F,K,K,E);
	   motionFromEssential (E,R,t);
     index_right_sol = motionFromEssentialChooseSolution (R,t,K,x1,K,x2);

     if (index_right_sol != -1) {
	   temp_t = t[index_right_sol];
	   temp_R = R[index_right_sol];
	 }
	 else {
       temp_t = Mat::zeros (3,1,CV_64F);
       temp_R = Mat::ones (3,3,CV_64F);
	 }
     }
     prev_t = ts;
     prev_R = Rs;
     temp_t = Rs*temp_t;
     Rs = temp_R * Rs;

     //test = temp_R*ts;
     add (ts,temp_t,ts);
     Mat T_Rs;
     transpose(temp_R,T_Rs);
     Cp = -T_Rs*temp_t;
     cout << "\nprinting coordinate of camera\n";
     for (int i=0;i<ts.rows;i++)
     cout << "\t"<< ts.at<double>(i,0);
     bool keyframe_selection = keyFrameSelection(Cp, keyframe_change);
     if (keyframe_selection == true)
     Cp = Mat::zeros (3,1,CV_64F);
     cout << "\nchecking keyframe selection creiteria\t" << keyframe_selection << "\n";


    x = int(ts.at<double>(0,0)) +300;
    y = int(ts.at<double>(2,0)) +300;
    circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

    //rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
    imshow( "Trajectory", traj );

     //if (keyframe_selection == true)
     //Cp = Mat::zeros (3,1,CV_64F);
     //Affine3f pose(rot_mat,Vec3d(ts.at <double>(1,0), ts.at <double>(2,0), ts.at <double>(3,0)));
     //Affine3f pose(Rs,ts);
     //myWindow.setWidgetPose("Cube Widget", pose);
     //myWindow.spinOnce(1, true);
     printf ("testing...");
     for (int i=0;i<ts.rows;i++) {
       cout << "\t" << temp_t.at<double>(i,0);
     }
	 printf ("\n\n\n");
     Mat points3d;
     cout << "\nprinting 3d point vector size\t" << points3d_set.size() << "\n";
	 // Triangulate points between two given frames
	 // TODO(MANAN): Separate as a function
     if (keyframe_selection == true) {

       cout << prev_features << "n" << curr_features;
       triangulateLandmarks (prev_features, curr_features, prev_R, prev_t, Rs, ts, points3d);
       cout << "\n printing points3d " << points3d << "\n";
       points3d_set.push_back(points3d);
       vector <Point2f> reprojectedPoints;
       //point_cloud_est.push_back(Vec3d(points3d));
       path.push_back(Affine3d(Rs,ts));

      /* for (int i=0;i<prev_t.rows;i++) {
         cout << "\n";
	     for (int j=0;j<prev_t.cols;j++) {
           cout << "\t" << prev_t.at <double> (i,j);
	     }
       }
       cout << "\n";
       for (int i=0;i<ts.rows;i++) {
         cout << "\n";
	     for (int j=0;j<ts.cols;j++) {
	       cout << "\t" << ts.at <double> (i,j);
	     }
       }
       cout << "\n" << index_right_sol << "\n";
       int l=0;*/
       // Reprojection of points
       // TODO(MANAN) : Separate out as another function
       /*for (int i=0;i<points3d.cols;i++) {
         Mat pt_ImageEuclidean;
         Mat pt_ImageHomogenous;
         Mat pt_WorldHomogenous;
         Mat pt_WorldEuclidean = points3d.col(i);
         cout << points3d.col(i) << "\t";
         euclideanToHomogeneous (pt_WorldEuclidean, pt_WorldHomogenous);
         cout << "\nprinting pt_world homogenous size\t" << pt_WorldHomogenous.rows << "\n";
         for (int i=0;i<pt_WorldHomogenous.rows;i++) {
           cout << "\n";
           for (int j=0;j<pt_WorldHomogenous.cols;j++) {
             cout << "\t" << pt_WorldHomogenous.at <double> (i,j);
           }
         }
         cout << "\nprinting P_currFrame size\t" << P_currFrame.rows << "\n";
         for (int i=0;i<P_currFrame.rows;i++) {
           cout << "\n";
           for (int j=0;j<P_currFrame.cols;j++) {
             cout << "\t" << P_currFrame.at <double> (i,j);
           }
         }
         cout << "\nprinting M size\t" << P_currFrame.t().size() << "\n";
         cout << "\nprinting K size\t" << K.size() << "\n";
         pt_ImageHomogenous = K*(P_currFrame)*(pt_WorldHomogenous);
         cout << "\ndebugging\n";
         homogeneousToEuclidean (pt_ImageHomogenous,pt_ImageEuclidean);
         Point2f pt;
         pt.x = pt_ImageEuclidean.at <double> (0,0);
         pt.y = pt_ImageEuclidean.at <double> (0,1);
         reprojectedPoints.push_back(pt);
         l +=1;
       }
       cout << "\nprinting reprojected points size\n" << reprojectedPoints.size();
       vector <KeyPoint> reproject_points;
       for (vector<Point2f>::const_iterator it = reprojectedPoints.begin();
		                             it != reprojectedPoints.end(); it++) {
         cv::KeyPoint kpr(*it, 8);
		 reproject_points.push_back(kpr);
       }
       cout << "\nprinting reprojected keypoints size\n" << reproject_points.size();
       Mat out_reproject;
       drawKeypoints (currFrame, reproject_points, out_reproject, Scalar::all(255));
       imshow ("reproject", out_reproject);
       cout << "\n";*/
       //break;
     }

     //cout << "\nprinting 3d point matrix size" << projectionMatrices.size();
     Points2d.push_back(prev_features);
     prevFrame = currFrame.clone();
     prev_features = curr_features;
     if (prev_features.size() < MIN_NUM_FEAT) {
      featureDetection(prevFrame, prev_features);
       featureTracking(prevFrame, currFrame, prev_features, curr_features, status);
     }
//-----------------------Testing OpenCV reconstruct module-----------------------------------------//
        /*if(n_views > 3){
        cout <<"printing point cloud ...";
        Mat points3d;
        Mat Rr;
        Mat tr;
        Mat k;
        reconstruct(Points2d,Rr,tr,k,points3d);
        cout<<"\ntesting for reconstruct\n";
        //for(int i=0;i<points3d.cols;i++)
        cout << points3d.rows <<"\n";
        }*/
//---------------------------------------------------------------------------------------------------//

     if (waitKey(30) == 32) break;
   }
   cout << "\n printing points3d_set size   " << points3d_set.at(0) << "\n";
    for (int i = 0; i < points3d_set.size(); ++i){
    cout << "\n TESTING POINT CLOUD GENERATION \n";
    point_cloud_est.push_back(Vec3d(points3d_set[i]));}
    //viz::WCloud cloud_widget(points3d_set, viz::Color::green());
     if ( point_cloud_est.size() > 0 )
  {
    viz::WCloud cloud_widget(point_cloud_est, viz::Color::green());
    window.showWidget("point_cloud", cloud_widget);
    }

    window.showWidget("cameras_frames_and_lines", viz::WTrajectory(path, viz::WTrajectory::BOTH, 0.1, viz::Color::green()));
       window.showWidget("cameras_frustums", viz::WTrajectoryFrustums(path, Kc, 0.1, viz::Color::yellow()));
       window.setViewerPose(path[0]);
       // Create a window for display.
       window.spin();
   waitKey(0);
   return 0;
 }

