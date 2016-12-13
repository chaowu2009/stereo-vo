/*

 The MIT License

 Copyright (c) 2015 Avi Singh

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.

 */

#include "opencv2/opencv_modules.hpp"
#include <stdio.h>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "opencv2/xfeatures2d.hpp"

#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

using namespace cv::xfeatures2d;

#define MIN_NUM_FEAT 200
#define MAX_FRAME 4541


double getAbsoluteScale(int frame_id, int sequence_id, double z_cal)  {
  
  string line;
  int i = 0;
  ifstream myfile ("/home/cwu/Downloads/dataset/sequences/00/calib.txt");
  double x =0, y=0, z = 0;
  double x_prev, y_prev, z_prev;
  if (myfile.is_open())
  {
    while (( getline (myfile,line) ) && (i<=frame_id))
    {
      z_prev = z;
      x_prev = x;
      y_prev = y;
      std::istringstream in(line);
      cout << line << '\n';
      for (int j=0; j<12; j++)  {
        in >> z ;
        if (j==7) y=z;
        if (j==3)  x=z;
      }
      
      i++;
    }
    myfile.close();
  }

  else {
    cout << "Unable to open file";
    return 0;
  }

  return sqrt((x-x_prev)*(x-x_prev) + (y-y_prev)*(y-y_prev) + (z-z_prev)*(z-z_prev)) ;

}

void featureTracking(Mat img_1, 
	                 Mat img_2, 
	                 vector<Point2f>& points1,
		             vector<Point2f>& points2, 
		             vector<uchar>& status) {

//this function automatically gets rid of points for which tracking fails

	vector<float> err;
	Size winSize = Size(21, 21);
	TermCriteria termcrit = TermCriteria(
			TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);

    // img_1, pointers1 previous frame
    // img_2, pointers2 previous frame
	calcOpticalFlowPyrLK(img_1,  img_2, points1, points2, status, err, winSize,
			3, termcrit, 0, 0.001);

	//getting rid of points for which the KLT tracking failed or those who have gone outside the frame
	int indexCorrection = 0;
	for (int i = 0; i < status.size(); i++) {
		Point2f pt = points2.at(i - indexCorrection);
		if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
			if ((pt.x < 0) || (pt.y < 0)) {
				status.at(i) = 0;
			}
			points1.erase(points1.begin() + (i - indexCorrection));
			points2.erase(points2.begin() + (i - indexCorrection));
			indexCorrection++;
		}

	}

}

//uses FAST as of now, modify parameters as necessary
void featureDetection(Mat img_1, vector<Point2f>& points1) {
	vector < KeyPoint > keypoints_1;
	int fast_threshold = 20;
	bool nonmaxSuppression = true;
	FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
	KeyPoint::convert(keypoints_1, points1, vector<int>());
}

void computeInitialPose(string folder, 
	                    Mat &R_f, 
	                    Mat &t_f, 
	                    Mat &img_2,
		                vector<Point2f> &points2) {

	Mat img_1;
	char filename1[200];
	char filename2[200];
	sprintf(filename1, "/home/cwu/Downloads/dataset/sequences/00/image_0/%06d.png", 0);
	sprintf(filename2, "/home/cwu/Downloads/dataset/sequences/00/image_0/%06d.png", 1);

	//read the first two frames from the dataset
	Mat img_1_c = imread(filename1);
	Mat img_2_c = imread(filename2);

	if (!img_1_c.data || !img_2_c.data) {
		std::cout << " --(!) Error reading images " << std::endl;
	}

	// we work with grayscale images
	cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
	cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

	// feature detection, tracking
	vector < Point2f > points1; //vectors to store the coordinates of the feature points
	featureDetection(img_1, points1);        //detect features in img_1
	vector < uchar > status;
	featureTracking(img_1, img_2, points1, points2, status); //track those features to img_2

	//TODO: add a fucntion to load these values directly from KITTI's calib files
	// WARNING: different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters
	double focal = 718.8560;
	cv::Point2d pp(607.1928, 185.2157);
	//recovering the pose and the essential matrix
	Mat E, R, t, mask;
	E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
	recoverPose(E, points2, points1, R, t, focal, pp, mask);

	R_f = R.clone();
	t_f = t.clone();

}

void computeInitialStereoPose(string folder, 
	                    Mat &R_f_left, 
	                    Mat &t_f_left, 
	                    Mat &img_left,
		                vector<Point2f> &points_left,
		                Mat &R_f_right, 
	                    Mat &t_f_right, 
		                Mat &img_right,
		                vector<Point2f> &points_right) {
    // left camera
	Mat img_1;
	char filename1[200];
	char filename2[200];
	sprintf(filename1, "/home/cwu/Downloads/dataset/sequences/00/image_0/%06d.png", 0);
	sprintf(filename2,	"/home/cwu/Downloads/dataset/sequences/00/image_0/%06d.png", 1);

	//read the first two frames from the dataset
	Mat img_1_c = imread(filename1);
	Mat img_2_c = imread(filename2);

	if (!img_1_c.data || !img_2_c.data) {
		std::cout << " --(!) Error reading images " << std::endl;
	}

	// we work with grayscale images
	cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
	cvtColor(img_2_c, img_left, COLOR_BGR2GRAY);

	// feature detection, tracking
	vector < Point2f > points1; //vectors to store the coordinates of the feature points
	featureDetection(img_1, points1);        //detect features in img_1
	vector < uchar > status;
	featureTracking(img_1, img_left, points1, points_left, status); //track those features to img_2

	//TODO: add a fucntion to load these values directly from KITTI's calib files
	// WARNING: different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters
	double focal = 718.8560;
	cv::Point2d pp(607.1928, 185.2157);
	//recovering the pose and the essential matrix
	Mat E, R, t, mask;
	E = findEssentialMat(points_left, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
	recoverPose(E, points_left, points1, R, t, focal, pp, mask);

	R_f_left = R.clone();
	t_f_left = t.clone();


   // right camera
	
	sprintf(filename1, "/home/cwu/Downloads/dataset/sequences/00/image_1/%06d.png", 0);
	sprintf(filename2,	"/home/cwu/Downloads/dataset/sequences/00/image_1/%06d.png", 1);

	//read the first two frames from the dataset
	img_1_c = imread(filename1);
	img_2_c = imread(filename2);

	if (!img_1_c.data || !img_2_c.data) {
		std::cout << " --(!) Error reading images " << std::endl;
	}

	// we work with grayscale images
	cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
	cvtColor(img_2_c, img_right, COLOR_BGR2GRAY);

	// feature detection, tracking
	//vector < Point2f > points1; //vectors to store the coordinates of the feature points
	featureDetection(img_1, points1);        //detect features in img_1
	//vector < uchar > status;
	featureTracking(img_1, img_left, points1, points_right, status); //track those features to img_2

	//recovering the pose and the essential matrix
	//Mat E, R, t, mask;
	E = findEssentialMat(points_right, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
	recoverPose(E, points_right, points1, R, t, focal, pp, mask);

	R_f_right = R.clone();
	t_f_right= t.clone();


}


void updatePose(char filename[100], 
	            Mat &prevImage,
		        vector<Point2f> &prevFeatures,
		        vector<Point2f> &currFeatures, 
		        Mat &R_f, 
		        Mat &t_f) 
{
	Mat currImage_c = imread(filename);
	Mat currImage; 
	cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
	vector < uchar > status;
	featureTracking(prevImage, currImage,  prevFeatures, currFeatures,  status);

	Mat E, R, t, mask;
	double focal = 718.8560;
	cv::Point2d pp(607.1928, 185.2157);
	E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999,	1.0, mask);
	recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

	Mat prevPts(2, prevFeatures.size(), CV_64F);
	Mat currPts(2, currFeatures.size(), CV_64F);

	//this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
	//for (int i = 0; i < prevFeatures.size(); i++) {
	//	prevPts.at<double>(0, i) = prevFeatures.at(i).x;
	//	prevPts.at<double>(1, i) = prevFeatures.at(i).y;

	//	currPts.at<double>(0, i) = currFeatures.at(i).x;
	//	currPts.at<double>(1, i) = currFeatures.at(i).y;
	//}

	//scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));

	//cout << "Scale is " << scale << endl;
	double scale = 0.90;

	if ((scale > 0.1) && (t.at<double>(2) > t.at<double>(0))
			&& (t.at<double>(2) > t.at<double>(1))) {

		t_f = t_f + scale * (R_f * t);
		R_f = R * R_f;

	}

	// a redetection is triggered in case the number of feautres being trakced go below a particular threshold
	if (prevFeatures.size() < MIN_NUM_FEAT) {
		featureDetection(prevImage, prevFeatures);
		featureTracking(currImage, prevImage, prevFeatures, currFeatures, status);

	}

	prevImage = currImage.clone();
	prevFeatures = currFeatures;
}

void stereoVisionIntialPose(string folder, 
		                    Mat &img_left, vector<Point2f> &feature_left,
		                    Mat &img_right, vector<Point2f> &feature_right) {

	// left camera
	char folder_left[100];
	Mat R_f_left, t_f_left;
	computeInitialPose(folder_left, R_f_left, t_f_left, img_left, feature_left);

	// right camera
	char folder_right[100];
	Mat R_f_right, t_f_right;
	computeInitialPose(folder_right, R_f_right, t_f_right, img_right,
			feature_right);
}

void stereoVision(string folder, 
	              int numFrame, 
	              Mat &currImage_lc, 
	              Mat &currImage_rc, 
	              Mat &previous_img_left, 
		          vector<Point2f> &previous_feature_left, 
		          vector<Point2f> &current_feature_left, 
		          Mat &previous_img_right, 
		          vector<Point2f> &previous_feature_right, 
		          vector<Point2f> &current_feature_right, 
		          Mat &R_f,
		          Mat &t_f) 
{

	double focal = 718.8560;
	cv::Point2d pp(607.1928, 185.2157);
	double scale = 0.95;

	char filename1[200], filename2[200];
	sprintf(filename1, "/home/cwu/Downloads/dataset/sequences/00/image_0/%06d.png", numFrame);
	sprintf(filename2, "/home/cwu/Downloads/dataset/sequences/00/image_0/%06d.png", numFrame);


	//read the first two frames from the dataset
	Mat current_img_left  = imread(filename1);
	currImage_lc = current_img_left;   //for plotting purpose

	Mat current_img_right = imread(filename2);
    currImage_rc = current_img_right;   //for plotting purpose

	if (!current_img_left.data || !current_img_right.data) {
	    std::cout << " --(!) Error reading images " << std::endl;
	}

	// we work with grayscale images
	cvtColor(current_img_left, current_img_left, COLOR_BGR2GRAY);
	cvtColor(current_img_right, current_img_right, COLOR_BGR2GRAY);

	// left camera feature tracking
	vector < uchar > status;
	featureTracking(previous_img_left, current_img_left, previous_feature_left,	current_feature_left, status);
    
    // left pose estimation
	Mat E1, R1, t1, mask1;
	E1 = findEssentialMat(current_feature_left, previous_feature_left, focal, pp, RANSAC, 0.999,	1.0, mask1);
	recoverPose(E1, current_feature_left, previous_feature_left, R1, t1, focal, pp, mask1);


	// right camera feature tracking
	featureTracking(previous_img_right, current_img_right, previous_feature_right, current_feature_right, status);	
    // right pose estimation
    Mat E2, R2, t2, mask2;
    E2 = findEssentialMat(current_feature_right, previous_feature_right, focal, pp, RANSAC, 0.999,	1.0, mask2);
	recoverPose(E2, current_feature_right, previous_feature_right, R2, t2, focal, pp, mask2);

    // fuse left and right pose
		
	Mat t = (t1+t2)/2.0;
	Mat R = R2 ;

	Mat detlaR = R2.inv()*R1;
	// normalize R
	double sum = 0;
	int cols = R.cols, rows= R.rows;

	// enlarge the diagnoal term
	for (int i = 0; i < rows; i++){
		double * Mp = detlaR.ptr<double>(i);
		for (int j = 0; j < cols; j++){
			if (i==i) {
		    	Mp[j] = Mp[i]*2;
		    }
		}
    }
   
   for (int i = 0; i < rows; i++){
		const double * Mp = detlaR.ptr<double>(i);
		for (int j = 0; j < cols; j++){
	        sum += Mp[j]*Mp[j];
		}
    }
   double sqrt_of_sum = sqrt(sum);

   // Normalize
   for (int i = 0; i < rows; i++){
		double * Mp = detlaR.ptr<double>(i);
		for (int j = 0; j < cols; j++){
			Mp[j] = Mp[j]/sqrt_of_sum;
		}
    }
   
 //  R= R1 * detlaR;
     scale = 0.8;
	if ( (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
	    t_f = t_f + scale * (R_f * t);
		R_f = R * R_f;
	}

    // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
	if (previous_feature_left.size() < MIN_NUM_FEAT) {
		featureDetection(previous_img_left, previous_feature_left);
		featureTracking(previous_img_left, current_img_left, previous_feature_left, current_feature_left, status);
	}

	previous_img_left = current_img_left.clone();
	previous_feature_left = current_feature_left;

	// a redetection is triggered in case the number of feautres being trakced go below a particular threshold
	if (previous_feature_right.size() < MIN_NUM_FEAT) {
		featureDetection(previous_img_right, previous_feature_right);
		featureTracking(previous_img_right, current_img_right, previous_feature_right, current_feature_right, status);
	}

	previous_img_right = current_img_right.clone();
	previous_feature_right= current_feature_right;

}

void stereoPoseUpdate(Mat &R_f, Mat &t_f, vector<Point2f> &previous_feature,
		vector<Point2f> &current_feature) {

	double focal = 718.8560;
	cv::Point2d pp(607.1928, 185.2157);
	double scale = 1.0;

	Mat E, R, t, mask;
	Mat t_f_left, R_f_left;

	E = findEssentialMat(current_feature, previous_feature, focal, pp, RANSAC,
			0.999, 1.0, mask);
	recoverPose(E, current_feature, previous_feature, R, t, focal, pp, mask);


}

// check features are matched or not between left and right.
bool MatchFeatures(Mat &img_1, Mat &img_2, Mat &R_f, Mat &t_f) {

	// ref: http://stackoverflow.com/questions/27533203/how-do-i-use-sift-in-opencv-3-0-with-c
	//-- Step 1: Detect the keypoints using SURF Detector
	//-- Step 1: Detect the keypoints using SURF Detector
	// detecting keypoints
	//SurfFeatureDetector detector(400);
	Ptr < SURF > detector = SURF::create(400);
	vector<KeyPoint> keypoints1, keypoints2;
	detector->detect(img_1, keypoints1);
	detector->detect(img_2, keypoints2);

	// computing descriptors
	//SurfDescriptorExtractor extractor;
	Ptr < SURF > extractor = SURF::create();
	Mat descriptors1, descriptors2;
	extractor->compute(img_1, keypoints1, descriptors1);
	extractor->compute(img_2, keypoints2, descriptors2);

	// matching descriptors
	BFMatcher matcher;
	std::vector < DMatch > matches;
	matcher.match(descriptors1, descriptors2, matches);

	// drawing the results
	namedWindow("matches", 1);
	Mat img_matches;
	drawMatches(img_1, keypoints1, img_2, keypoints2, matches, img_matches);
	imshow("matches", img_matches);
	waitKey(0);

}
