#include "rotation.h"
#include "vo_features.h"
#include <fstream>
#include <ostream>
#include <string>

#ifdef __linux__ 
#include "readBNO.h"
#endif

using namespace cv;
using namespace std;


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

int main(int argc, char** argv) {

	clock_t begin = clock();
	clock_t currentFrameClock;
	float deltaTinSecond = 0.0f;
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
	string imgDir = "d:/vision/dataset/sequences/5/";
	string resultFile = imgDir + "vo_result.txt";
	std::string imgFormat = ".jpg";
	std::string quaternionFile = imgDir + "q.txt";
	std::fstream infile(quaternionFile);
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
	Mat currImage_lc;

	Mat R_f, t_f; //the final rotation and translation vectors

	Mat img_1, img_2;  // two consecutive images from the same camera

	cout << "Running at simulation mode!" << endl;
	vector < Point2f > points1, points2; //vectors to store the coordinates of the feature points
	
    
	loadImage(imgDir + "/img_left/1" + imgFormat, temp);
	rectifyImage(temp, previous_img);

	loadImage(imgDir + "/img_left/2" + imgFormat, temp);
	rectifyImage(temp, current_img);

	// features
	vector < Point2f > keyFeatures;

	featureDetection(previous_img, keyFeatures);        //detect features in img_1
	vector < Point2f > prevFeatures = keyFeatures;
	
	featureDetection(current_img, keyFeatures);        //detect features in img_1
	vector < Point2f > currFeatures = keyFeatures;

	string filename;
	Mat E, R, t, mask;

    vector < uchar > status;
	featureTracking(previous_img, current_img, prevFeatures, currFeatures, status);

	E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
	recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

	R_f = R.clone();
	t_f = t.clone();

	namedWindow("Trajectory", WINDOW_AUTOSIZE); // Create a window for display.

	Mat traj = Mat::zeros(640, 480, CV_8UC3);
	Mat trajTruth = Mat::zeros(640, 480, CV_8UC3);

	for (int numFrame = 3; numFrame < MAX_FRAME; numFrame++) {
		cout << "numFrame = " << numFrame << endl;

		stringstream ss;
		ss << numFrame;
		string idx = ss.str();

		string filename1 = imgDir + "/img_left/" + idx + imgFormat;

		loadImage(filename1, temp);

		rectifyImage(temp, current_img);

		//updatePose(filename,	previous_img,		prevFeatures,		currFeatures,		R_f,		t_f);
		featureTracking(previous_img, current_img, prevFeatures, currFeatures, status);
		previous_img = current_img.clone();

		E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
		recoverPose(E, currFeatures, prevFeatures, R_f, t_f, focal, pp, mask);

		if ((t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
			cout << "computing pose" << endl;
			t_f = t_f + scale * (R_f * t);
			R_f = R * R_f;

		}

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

		waitKey(2);  //microsecond
	}

	imwrite(imgDir + "/final_map" + imgFormat, traj);

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Total time taken: " << elapsed_secs << "s" << endl;

	return 0;
}
