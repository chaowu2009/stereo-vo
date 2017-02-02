
#include "vo_features.h"

using namespace cv;
using namespace std;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000
#define LEFT 0
#define RIGHT 1

//#define REAL_TIME

int main(int argc, char** argv) {

	Mat img_1, img_2;
	
	Mat R_f, t_f; //the final rotation and tranlation vectors containing the 

	ofstream myfile;
	myfile.open("results1_1.txt");

#ifdef REAL_TIME

	cout << "Running at real-time" << endl;
	
	VideoCapture left_capture(LEFT);
	left_capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	left_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);

	left_capture.read(img_1);
	cvtColor(img_1, img_1, COLOR_BGR2GRAY);
	
	left_capture.read(img_2);
	cvtColor(img_2, img_2, COLOR_BGR2GRAY);


#else
	double scale = 1.00;
	char filename1[200];
	char filename2[200];
	sprintf(filename1, "D:/vision/dataset/sequences/00/image_1/%06d.png", 0);
	sprintf(filename2, "D:/vision/dataset/sequences/00/image_1/%06d.png", 1);

	char text[100];
	int fontFace = FONT_HERSHEY_PLAIN;
	double fontScale = 1;
	int thickness = 1;
	cv::Point textOrg(10, 50);

	//read the first two frames from the dataset
	img_1 = imread(filename1);
	img_2 = imread(filename2);
#endif
	if (!img_1.data || !img_2.data) {
		std::cout << " --(!) Error reading images " << std::endl; return -1;
	}

	cvtColor(img_1, img_1, COLOR_BGR2GRAY);
	cvtColor(img_2, img_2, COLOR_BGR2GRAY);

	// feature detection, tracking
	vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points
	featureDetection(img_1, points1);        //detect features in img_1
	vector<uchar> status;
	featureTracking(img_1, img_2, points1, points2, status); //track those features to img_2

	//recovering the pose and the essential matrix
	Mat E, R, t, mask;
	E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
	recoverPose(E, points2, points1, R, t, focal, pp, mask);

	Mat prevImage = img_2;
	Mat currImage;
	vector<Point2f> prevFeatures = points2;
	vector<Point2f> currFeatures;

	char filename[100];

	R_f = R.clone();
	t_f = t.clone();

	clock_t begin = clock();

	namedWindow("Road facing camera", WINDOW_AUTOSIZE);// Create a window for display.
	namedWindow("Trajectory", WINDOW_AUTOSIZE);// Create a window for display.

	Mat traj = Mat::zeros(600, 600, CV_8UC3);

	for (int numFrame = 2; numFrame < MAX_FRAME; numFrame++) {
#ifdef REAL_TIME
		left_capture.read(img_1);
#else
		sprintf(filename, "D:/vision/dataset/sequences/00/image_1/%06d.png", numFrame);
		Mat img_1 = imread(filename);
#endif
		cvtColor(img_1, currImage, COLOR_BGR2GRAY);
		vector<uchar> status;
		featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

		E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
		recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

		Mat prevPts(2, prevFeatures.size(), CV_64F), currPts(2, currFeatures.size(), CV_64F);
		
		for (int i = 0; i<prevFeatures.size(); i++) {   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
			prevPts.at<double>(0, i) = prevFeatures.at(i).x;
			prevPts.at<double>(1, i) = prevFeatures.at(i).y;

			currPts.at<double>(0, i) = currFeatures.at(i).x;
			currPts.at<double>(1, i) = currFeatures.at(i).y;
		}

		if ((t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {

			t_f = t_f + (R_f*t);
			R_f = R*R_f;

		}

		else {
			//cout << "scale below 0.1, or incorrect translation" << endl;
		}

		// lines for printing results
		// myfile << t_f.at<double>(0) << " " << t_f.at<double>(1) << " " << t_f.at<double>(2) << endl;

		// a redetection is triggered in case the number of feautres being trakced go below a particular threshold
		if (prevFeatures.size() < MIN_NUM_FEAT) {
			//cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
			//cout << "trigerring redection" << endl;
			featureDetection(prevImage, prevFeatures);
			featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

		}

		prevImage = currImage.clone();
		prevFeatures = currFeatures;

		int x = int(t_f.at<double>(0)) + 300;
		int y = int(t_f.at<double>(2)) + 100;
		circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);

		rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0), CV_FILLED);
		sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
		putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

		imshow("Road facing camera", currImage);
		imshow("Trajectory", traj);

		waitKey(1);

	}

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Total time taken: " << elapsed_secs << "s" << endl;

	//cout << R_f << endl;
	//cout << t_f << endl;

	return 0;
}