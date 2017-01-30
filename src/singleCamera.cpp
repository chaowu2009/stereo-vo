#include "rotation.h"
#include "vo_features.h"
#include <fstream>
#include <ostream>
#include <string>
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

//#define MIN_NUM_FEAT 200
#define PLOT_COLOR CV_RGB(0, 0, 0)
#define PL std::setprecision(3)

double scale = 1.00;
char text[100];
int fontFace = FONT_HERSHEY_PLAIN;
double fontScale = 1;
int thickness = 1;
cv::Point textOrg(10, 50);

int LEFT = 0;
int RIGHT = 1;
#if 1
int main(int argc, char** argv) {

	clock_t begin = clock();
	clock_t currentFrameClock;
	int MAX_FRAME = 1000;

	float q[4];
	q[0] = 1.0; q[1] = 0.0; q[2] = 0.0; q[3] = 0.0;
	Mat dcm;
	dcm = (cv::Mat_<float>(3, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);

#ifdef __linux__ 
	int fd = initBNO();

	readQuaternion(fd, q);  //the first sample is bad. So clean the buffer
#endif

#ifdef __linux__ 
							//linux code goes here

	std::string imgDir = "/home/cwu/project/dataset/images/4";
	std::string imgFormat = ".jpg";
	std::string timeStampFile = imgDir + "timeStamp.txt";
	std::string resultFile = imgDir + "vo_result.txt";

	MAX_FRAME = 1000;

	std::string quaternionFile = imgDir + "q.txt";
	std::fstream infile(quaternionFile.c_str());
	std::string line;
#else
							// windows code goes here
	//string imgDir = "d:/vision/dataset/sequences/planar/";
	string imgDir = "d:/vision/dataset/sequences/00/image_0/";
	string resultFile = "vo_result.txt";

	std::string imgFormat = ".png";
	//std::string quaternionFile = imgDir + "q.txt";
	//std::fstream infile(quaternionFile);
	std::string line;
#endif

	// Open a txt file to store the results
	ofstream fout(resultFile.c_str());
	if (!fout) {
		cout << "Result file not opened!" << endl;
		return 1;
	}

	Mat current_img, previous_img, temp;

	// for plotting purpose 
	Mat R_f, t_f; //the final rotation and translation vectors

	cout << "Running at simulation mode!" << endl;

	loadImage(imgDir + "000000" + imgFormat, previous_img);
	//rectifyImage(temp, previous_img);
	 
	loadImage(imgDir + "000001" + imgFormat, current_img);
//	rectifyImage(temp, current_img);

	// features
	vector < Point2f > prevFeatures, currFeatures, keyFeatures;

	featureDetection(previous_img, prevFeatures);        //detect features in img_1
	
	featureDetection(current_img, currFeatures);        //detect features in img_2
	
	string filename;
	Mat E, R, t, mask;

	vector < uchar > status;
	featureTracking(previous_img, current_img, prevFeatures, currFeatures, status);
	//cout << "prevFeatures numbers = " << prevFeatures.size() << endl;
	//cout << "currFeatures numbers = " << currFeatures.size() << endl;

	E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
	recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

	R_f = R.clone();
	t_f = t.clone();

	namedWindow("Trajectory", WINDOW_AUTOSIZE); // Create a window for display.

	Mat traj = Mat::zeros(640, 480, CV_8UC3);
	
	for (int numFrame = 3; numFrame < MAX_FRAME+1; numFrame++) {

		stringstream ss;
		ss << numFrame;
		string idx = ss.str();

		//string filename1 = imgDir + "/img_left/" + idx + imgFormat;
		string filename1 = combineName(imgDir, numFrame, imgFormat);

		loadImage(filename1, current_img);

	//	rectifyImage(temp, current_img);
		 
		featureDetection(current_img, currFeatures);        //detect features in img_1
		keyFeatures = currFeatures;
		//cout << "numFrame = " << numFrame << " currFeatures numbers = " << currFeatures.size() << endl;
		cout << "numFrame = " << numFrame  << endl;

		featureTracking(previous_img, current_img, prevFeatures, currFeatures, status);
		int anyFeatureTracked = 0;
		for (uint i = 0; i < status.size(); i++) {
			if (status[i] == 1)	anyFeatureTracked++;
		}
	//	cout << "numFrame = " << numFrame << " currFeatures numbers = " << currFeatures.size() << " feature tracked =" << anyFeatureTracked << endl;
		if (anyFeatureTracked < 10) {
			// copy for next loop
		//	previous_img = current_img.clone();
		//	prevFeatures = keyFeatures;
		//	continue; 
		}

		E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
		recoverPose(E, currFeatures, prevFeatures, R_f, t_f, focal, pp, mask);

        // update pose
		t_f = t_f + scale * (R_f * t);
		R_f = R * R_f;

		// copy for next loop
		previous_img = current_img;
		prevFeatures = currFeatures;
		
		// for plotting purpose
		double x1 = t_f.at<double>(0);
		double y1 = t_f.at<double>(1);
		double z1 = t_f.at<double>(2);

		int x = int(x1) + 320;
		int y = int(z1) + 240;

		// output to the screen
		cout << "numFrame:" << numFrame << " vo: x = " << PL << x1 << "\t y = " << PL << y1 << "\t z = " << PL << z1 << endl;
		// current point
		circle(traj, Point(x, y), 0.2, CV_RGB(255, 0, 0), 2);
		rectangle(traj, Point(10, 30), Point(500, 50), PLOT_COLOR, CV_FILLED);

		sprintf(text, "Coordinates: x = %04fm y = %04fm z = %04fm", x1, y1, z1);
		putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

		// plot them
		imshow("Left camera", current_img);
		imshow("Trajectory", traj);

		// Save the result
		fout << numFrame << "\t";
		fout << x1 << "\t" << y1 << "\t" << z1 << "\t" << x << "\t" << y << "\n";

		waitKey(1);  //microsecond
	}

	imwrite(imgDir + "/final_map" + imgFormat, traj);

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Total time taken: " << elapsed_secs << "s" << endl;

	return 0;
}

#else

void readme();

/** @function main */
int main(int argc, char** argv)
{
	Mat temp, previous_img, current_img;
	string imgDir = "d:/vision/dataset/sequences/planar_lower/";
	std::string imgFormat = ".jpg";

	int MAX_FRAME = 1000;

	Mat img_1 = imread(imgDir + "/img_left/1" + imgFormat, IMREAD_GRAYSCALE);

	if (!img_1.data)
	{
		std::cout << " --(!) Error reading images " << std::endl; return -1;
	}


	//-- Step 1: Detect the keypoints using SURF Detector
	int minHessian = 400;

	Ptr<SURF> detector = SURF::create(minHessian);

	std::vector<KeyPoint> keypoints_1;

	for (int numFrame = 3; numFrame < MAX_FRAME; numFrame++) {


		stringstream ss;
		ss << numFrame;
		string idx = ss.str();

		string filename1 = imgDir + "/img_left/" + idx + imgFormat;


		img_1 = imread(filename1, IMREAD_GRAYSCALE);


		detector->detect(img_1, keypoints_1);

		cout << "numFrame = " << numFrame << " numOfKeyPoints =" << keypoints_1.size() << endl;

		//-- Draw keypoints
		Mat img_keypoints_1;

		drawKeypoints(img_1, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

		//-- Show detected (drawn) keypoints
		imshow("Keypoints 1", img_keypoints_1);


		waitKey(1);
	}

	return 0;
}

/** @function readme */
void readme()
{
	std::cout << " Usage: ./SURF_detector <img1> <img2>" << std::endl;
}

#endif