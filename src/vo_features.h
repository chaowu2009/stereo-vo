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
#include <algorithm> 
#include <iterator> 
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <cv.h>
#include <stdarg.h>
#include <iomanip>

#if defined(__linux__)
#  include <unistd.h>
#elif defined(_WIN32)
#  include <windows.h>
#  define sleep(s) Sleep((s)*1000)
#endif

using namespace cv;
using namespace std;

using namespace cv::xfeatures2d;

#define MIN_NUM_FEAT 200

extern const double focal;
extern const cv::Point2d pp;

double getAbsoluteScale(int frame_id, int sequence_id, double z_cal)  {
  
  string line;
  int i = 0;
  ifstream myfile ("D:/vision/dataset/sequences/00/calib.txt");
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


void getPosition_Orientation(string line, float vout[3], Mat &Rout ){
	// If possible, always prefer std::vector to naked array
	std::vector<float> v;

	// Build an istream that holds the input string
	std::istringstream iss(line);

	// Iterate over the istream, using >> to grab floats
	// and push_back to store them in the vector
	std::copy(std::istream_iterator<float>(iss),
		std::istream_iterator<float>(),
		std::back_inserter(v));

	vout[0] = v[3];
	vout[1] = v[7];
	vout[2] = v[11];

	Rout.at<double>(0,0) = v[0];
	Rout.at<double>(0,1) = v[1];
	Rout.at<double>(0,2) = v[2];
	Rout.at<double>(1,0) = v[4];
	Rout.at<double>(1,1) = v[5];
	Rout.at<double>(1,2) = v[6];
	Rout.at<double>(2,0) = v[8];
	Rout.at<double>(2,1) = v[9];
	Rout.at<double>(2,2) = v[10];

}

void getPosition(string line, float vout[3]){
	// If possible, always prefer std::vector to naked array
	std::vector<float> v;

	// Build an istream that holds the input string
	std::istringstream iss(line);

	// Iterate over the istream, using >> to grab floats
	// and push_back to store them in the vector
	std::copy(std::istream_iterator<float>(iss),
		std::istream_iterator<float>(),
		std::back_inserter(v));

	vout[0] = v[3];
	vout[1] = v[7];
	vout[2] = v[11];

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
	for (uint i = 0; i < status.size(); i++) {
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

void computeInitialPose(Mat &img_1, 
	                    Mat &R_f, 
	                    Mat &t_f, 
	                    Mat &img_2,
		                vector<Point2f> &points2) {

	// feature detection, tracking
	vector < Point2f > points1; //vectors to store the coordinates of the feature points
	featureDetection(img_1, points1);        //detect features in img_1
	vector < uchar > status;
	featureTracking(img_1, img_2, points1, points2, status); //track those features to img_2

	//TODO: add a fucntion to load these values directly from KITTI's calib files
	// WARNING: different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters
	//recovering the pose and the essential matrix
	Mat E, R, t, mask;
	E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
	recoverPose(E, points2, points1, R, t, focal, pp, mask);

	R_f = R.clone();
	t_f = t.clone();

}

void computeInitialStereoPose(Mat &previous_img_left,
	                          Mat &current_img_left,
	                          Mat &R_f_left, 
	                          Mat &t_f_left, 
		                      vector<Point2f> &points_left,
		                      Mat &previous_img_right,
							  Mat &current_img_right,
							  Mat &R_f_right, 
	                          Mat &t_f_right, 
		                      vector<Point2f> &points_right) {
    
	// feature detection, tracking
	vector < Point2f > points1; //vectors to store the coordinates of the feature points
	featureDetection(previous_img_left, points1);        //detect features in img_1
	vector < uchar > status;
	featureTracking(previous_img_left, current_img_left, points1, points_left, status); //track those features to img_2

	// WARNING: different sequences in the KITTI VO dataset have different intrinsic/extrinsic parameters
	//recovering the pose and the essential matrix
	Mat E, R, t, mask;
	E = findEssentialMat(points_left, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
	recoverPose(E, points_left, points1, R, t, focal, pp, mask);

	R_f_left = R.clone();
	t_f_left = t.clone();
	
   // right camera
	// feature detection, tracking
	featureDetection(previous_img_right, points1);        //detect features in img_1
	featureTracking(previous_img_right, current_img_right, points1, points_right, status); //track those features to img_2

	//recovering the pose and the essential matrix
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

void stereoVisionIntialPose(Mat &img_left, vector<Point2f> &feature_left,
		                    Mat &img_right, vector<Point2f> &feature_right) {

	// left camera
	Mat R_f_left, t_f_left;
	computeInitialPose(img_left, R_f_left, t_f_left, img_left, feature_left);

	// right camera
	Mat R_f_right, t_f_right;
	computeInitialPose(img_right, R_f_right, t_f_right, img_right,
			feature_right);
}

void stereoVision(Mat &current_img_left,
	              Mat & current_img_right,
	              Mat &currImage_lc, 
	              Mat &currImage_rc, 
	              Mat &previous_img_left, 
		          vector<Point2f> &previous_feature_left, 
		          vector<Point2f> &current_feature_left, 
		          Mat &previous_img_right, 
		          vector<Point2f> &previous_feature_right, 
		          vector<Point2f> &current_feature_right, 
		          Mat &R_f,
		          Mat &t_f,
	              Mat &dcm) 
{
	double scale = 1.0;

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
   
   //R = dcm;
   //R= R1 * detlaR;
    
	if ( (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
		//R_f = dcm;
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

	double scale = 1.0;

	Mat E, R, t, mask;
	Mat t_f_left, R_f_left;

	E = findEssentialMat(current_feature, previous_feature, focal, pp, RANSAC,
			0.999, 1.0, mask);
	recoverPose(E, current_feature, previous_feature, R, t, focal, pp, mask);


}

// check features are matched or not between left and right.
void MatchFeatures(Mat &img_1, Mat &img_2, Mat &R_f, Mat &t_f) {

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

//https://github.com/opencv/opencv/wiki/DisplayManyImages
void cvShowManyImages(char* title, int nArgs, ...) {

	// img - Used for getting the arguments
	IplImage *img;

	// [[DispImage]] - the image in which input images are to be copied
	IplImage *DispImage;

	int size;
	int i;
	int m, n;
	int x, y;

	// w - Maximum number of images in a row
	// h - Maximum number of images in a column
	int w, h;

	// scale - How much we have to resize the image
	float scale;
	int max;

	// If the number of arguments is lesser than 0 or greater than 12
	// return without displaying
	if(nArgs <= 0) {
	printf("Number of arguments too small....\n");
	return;
	}
	else if(nArgs > 12) {
	printf("Number of arguments too large....\n");
	return;
	}
	// Determine the size of the image,
	// and the number of rows/cols
	// from number of arguments
	else if (nArgs == 1) {
	w = h = 1;
	size = 300;
	}
	else if (nArgs == 2) {
	w = 2; h = 1;
	size = 300;
	}
	else if (nArgs == 3 || nArgs == 4) {
	w = 2; h = 2;
	size = 300;
	}
	else if (nArgs == 5 || nArgs == 6) {
	w = 3; h = 2;
	size = 200;
	}
	else if (nArgs == 7 || nArgs == 8) {
	w = 4; h = 2;
	size = 200;
	}
	else {
	w = 4; h = 3;
	size = 150;
	}

	// Create a new 3 channel image
	//[[DispImage]] = cvCreateImage( cvSize(100 + size*w, 60 + size*h), 8, 3 );
	DispImage = cvCreateImage( cvSize(100 + size*w, 60 + size*h), 8, 3 );

	// Used to get the arguments passed
	va_list args;
	va_start(args, nArgs);

	// Loop for nArgs number of arguments
	for (i = 0, m = 20, n = 20; i < nArgs; i++, m += (20 + size)) {

	// Get the Pointer to the IplImage
	img = va_arg(args, IplImage*);

	// Check whether it is NULL or not
	// If it is NULL, release the image, and return
	if(img == 0) {
	printf("Invalid arguments");
	cvReleaseImage(&DispImage);
	return;
	}

	// Find the width and height of the image
	x = img->width;
	y = img->height;

	// Find whether height or width is greater in order to resize the image
	max = (x > y)? x: y;

	// Find the scaling factor to resize the image
	scale = (float) ( (float) max / size );

	// Used to Align the images
	if( i % w == 0 && m!= 20) {
	m = 20;
	n+= 20 + size;
	}

	// Set the image ROI to display the current image
	cvSetImageROI(DispImage, cvRect(m, n, (int)( x/scale ), (int)( y/scale )));

	// Resize the input image and copy the it to the Single Big Image
	cvResize(img, DispImage);

	// Reset the ROI in order to display the next image
	cvResetImageROI(DispImage);
	}

	// Create a new window, and show the Single Big Image
	cvNamedWindow( title, 1 );
	cvShowImage( title, DispImage);

	cvWaitKey();
	cvDestroyWindow(title);

	// End the number of arguments
	va_end(args);

	// Release the Image Memory
	cvReleaseImage(&DispImage);
}

// add a delay
void addDelay( float N){

	for(int k= 0; k< N; k++){}

}

void getImage(VideoCapture &capture, Mat &imgOut, Mat &edges)
{
	if(!capture.isOpened()){
		cout << "camera is not loaded correctly" << endl;
	}
		
	Mat currentFrame;
	capture >> currentFrame; // get a new frame from camera
	cvtColor(currentFrame, imgOut, COLOR_BGR2GRAY);

	// filter image
	//Mat imgTemp;
	//GaussianBlur(imgOut, imgTemp, Size(7,7), 1.5, 1.5);
	//Canny(imgTemp, edges, 0, 30, 3);
}

void loadImage(string fileName, Mat &imgOut, Mat &img_1_c){
	
	//read the image
//    cout << "image name = " << fileName << endl;
	img_1_c = imread(fileName);
	
	if (!img_1_c.data) {
		std::cout << " --(!) Error reading images " << std::endl;
	}

	// we work with grayscale images
	cvtColor(img_1_c, imgOut, COLOR_BGR2GRAY);
}

string combineName(string localDataDir, int numFrame){
	
	 std::ostringstream ostr;

	 ostr << std::setfill('0') << std::setw(6) << numFrame ;

	string fileName  = localDataDir +  ostr.str() + ".png";

	return fileName;

}  

void rectifyStereoImage(Mat &img1,
	              Mat &img2,
	              Mat &imgR1,
	              Mat &imgR2 ){

  Mat R1, R2, P1, P2, Q;
  Mat K1, K2, R;
  Vec3d T;
  Mat D1, D2;
//  char *calib_file ="cam_stereo.yml";

  //cv::FileStorage fs1("cam_stereo.yml", cv::FileStorage::READ);
#ifdef __linux__ 
  cv::FileStorage fs1("/home/cwu/project/stereo-vo/src/cam_stereo.yml", cv::FileStorage::READ);
#else
  cv::FileStorage fs1("d:/vision/stereo-vo/src/cam_stereo.yml", cv::FileStorage::READ);
#endif

  if (!fs1.isOpened()) { cout << "unable to open yml file" << endl; }
  fs1["K1"] >> K1;
  fs1["K2"] >> K2;
  fs1["D1"] >> D1;
  fs1["D2"] >> D2;
  fs1["R"] >> R;
  fs1["T"] >> T;

  fs1["R1"] >> R1;
  fs1["R2"] >> R2;
  fs1["P1"] >> P1;
  fs1["P2"] >> P2;
  fs1["Q"] >> Q;

  //cout << "Q = " << Q << endl;

  cv::Mat lmapx, lmapy, rmapx, rmapy;

  cv::initUndistortRectifyMap(K1, D1, R1, P1, img1.size(), CV_32F, lmapx, lmapy);
  cv::initUndistortRectifyMap(K2, D2, R2, P2, img2.size(), CV_32F, rmapx, rmapy);
  cv::remap(img1, imgR1, lmapx, lmapy, cv::INTER_LINEAR);
  cv::remap(img2, imgR2, rmapx, rmapy, cv::INTER_LINEAR);

}

void rectifyImage(Mat &imgIn,
	              Mat &imgOut)
{

  Mat R1, R2, P1, P2, Q;
  Mat K1, K2, R;
  Vec3d T;
  Mat D1, D2;
//  char *calib_file ="cam_stereo.yml";

  //cv::FileStorage fs1("cam_stereo.yml", cv::FileStorage::READ);
#ifdef __linux__ 
  cv::FileStorage fs1("/home/cwu/project/stereo-vo/src/cam_left.yml", cv::FileStorage::READ);
#else
  cv::FileStorage fs1("d:/vision/stereo-vo/src/cam_left.yml",cv::FileStorage::READ);
#endif

  if (!fs1.isOpened()) { cout << "unable to open yml file" << endl; }
  fs1["K"] >> K;
  fs1["D"] >> D;

  cv::Mat lmapx, lmapy, rmapx, rmapy;

  cv::initUndistortRectifyMap(K1, D1, R1, P1, img1.size(), CV_32F, lmapx, lmapy);
  cv::remap(imgIn, imgOut, lmapx, lmapy, cv::INTER_LINEAR);

}

 void q2Dcm(float q[4], Mat &dcm) {
	float q00, q01, q02, q03, q11, q12, q13, q22, q23, q33;
	float w, x, y, z;
	w = q[0];
	x = q[1];
	y = q[2];
	z = q[3];
	q00 = w*w;
	q01 = w* x;
	q02 = w * y;
	q03 = w* z;
	q11 = x*x;
	q12 = x* y;
	q13 = x* z;
	q22 = y*y;
	q23 = y* z;
	q33 = z*z;

	float DCM[3][3];
	DCM[0][0] = q00 + q11 - q22 - q33;
	DCM[1][0] = (q12 - q03)*2.0f;
	DCM[2][0] = (q13 + q02) *2.0f;

	DCM[0][1] = (q12 + q03)*2.0f;
	DCM[1][1]  = q00 - q11 + q22 - q33;
	DCM[2][1] = (q23 - q01)*2.0f;

	DCM[0][2] = (q13 - q02 )* 2.0f;
	DCM[1][2] = (q23 + q01)*2.0f;
	DCM[2][2] = q00 - q11 - q22 + q33;
	//cout << "DCM[0][0] = " << DCM[0][0] << endl;
	dcm = (cv::Mat_<float>(3, 3) << DCM[0][0], DCM[0][1], DCM[0][2], DCM[1][0], DCM[1][1], DCM[1][2], DCM[2][0], DCM[2][1], DCM[2][2]);
	//cout << "dcm = " << dcm << endl;
}

 void dcm2q(Mat dcm, float q[4]) {
	 float wSq4, xSq4, ySq4, zSq4; // sometimes <0
	 float DCM[3][3];
	 for (int i = 0; i < 3; i++)
		 for (int j = 0; j < 3; j++) {
			 DCM[i][j] = dcm.at<float>(i, j);
		 }
	 
	 wSq4 = 1 + DCM[0][0] + DCM[1][1] + DCM[2][2];
	 xSq4 = 1 + DCM[0][0] - DCM[1][1] - DCM[2][2];
	 ySq4 = 1 - DCM[0][0] + DCM[1][1] - DCM[2][2];
	 zSq4 = 1 - DCM[0][0] - DCM[1][1] + DCM[2][2];

	 float w2, x2, y2, z2;
	 if ((wSq4 >= xSq4) && (wSq4 >= ySq4) && (wSq4 >= zSq4)) { // wSq4 is the largest
		 w2 = sqrt(wSq4);
		 x2 = (DCM[2][1] - DCM[1][2]) / w2;
		 y2 = -(DCM[2][0] - DCM[0][2]) / w2;
		 z2 = (DCM[1][0] - DCM[0][1]) / w2;

	 }
	 else if ((xSq4 >= ySq4) && (xSq4 >= zSq4)) { // xSq4 is the largest
		 x2 = sqrt(xSq4);
		 w2 = (DCM[2][1] - DCM[1][2]) / x2;
		 y2 = (DCM[1][0] + DCM[0][1]) / x2;
		 z2 = (DCM[2][0] + DCM[0][2]) / x2;

	 }
	 else if (ySq4 >= zSq4) { // ySq4 is the largest
		 y2 = sqrt(ySq4);
		 w2 = -(DCM[2][0] - DCM[0][2]) / y2;
		 x2 = (DCM[1][0] + DCM[0][1]) / y2;
		 z2 = (DCM[2][1] + DCM[1][2]) / y2;

	 }
	 else { // zSq4 is the largest
		 z2 = sqrt(zSq4);
		 w2 = (DCM[1][0] - DCM[0][1]) / z2;
		 x2 = (DCM[2][0] + DCM[0][2]) / z2;
		 y2 = (DCM[2][1] + DCM[1][2]) / z2;
	 }

	 q[0] = w2 / 2.0f;
	 q[1] = x2 / 2.0f;
	 q[2] = y2 / 2.0f;
	 q[3] = z2/ 2.0f;
 }

