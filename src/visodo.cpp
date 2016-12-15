#include "vo_features.h"
#include <iomanip>
#include <fstream>

using namespace cv;
using namespace std;

#define MAX_FRAME 4541
#define MIN_NUM_FEAT 200
#define PLOT_COLOR CV_RGB(0, 0, 0)
#define PL std::setprecision(3)

// IMP: Change the file directories (4 places) according to where your dataset is saved before running!

double focal = 718.8560;
cv::Point2d pp(607.1928, 185.2157);

double scale = 1.00;
char text[100];
int fontFace = FONT_HERSHEY_PLAIN;
double fontScale = 1;
int thickness = 1;
cv::Point textOrg(10, 50);

//#define REAL_TIME

int main(int argc, char** argv) {

	#ifdef __linux__ 
    //linux code goes here
	string localDataDir = "/home/cwu/Downloads";
	string resultFile = "/home/cwu/project/stereo-vo/src/vo_result.txt";
    #else
    // windows code goes here
	string localDataDir = "d:/vision";
	string resultFile   = "d:/vision/stereo-vo/src/vo_result.txt";
    #endif

	cout << "localDataDir is " << localDataDir << endl;

	Mat current_img_left, current_img_right;
	Mat previous_img_left, previous_img_right; 
	Mat leftEdge, rightEdge;
    
	// for plotting purpose
	Mat currImage_lc, currImage_rc; 

	Mat R_f, t_f; //the final rotation and translation vectors
	
	Mat img_1, img_2;  // two consecutive images from the same camera

#ifdef REAL_TIME
	getImage(0, img_1, leftEdge);
#endif

	//obtain truth for plot comparison
	string posePath = localDataDir + "/dataset/poses/00.txt";
	std::ifstream infile(posePath.c_str());

	std::string line; 
	float truthPosition[3] ;
	Mat truthOrientation;

	getline(infile, line);
	getPosition(line, truthPosition);

	// Open a txt file to store the results
	ofstream fout(resultFile.c_str());
	if (!fout) {
		cout << "File not opened!" << endl;
		return 1;
	}
	
	// features
	vector < Point2f > keyFeatures;
	
#ifdef REAL_TIME
		
	//left camera
	// get the second frame
	getImage(0, img_2, leftEdge);
	
	getImage(1, previous_img_right, rightEdge);
	
	// display them
	//cvShowManyImages("Left & Right", 2, current_img_left, current_img_right);
	//imshow("Left Camera", leftCameraMat);
	//imshow("Right Camera", rightCameraMat);

#else
	// use the first two images from left camera to compute the init values.
	loadImage(localDataDir + "/dataset/sequences/00/image_0/000000.png", img_1, currImage_lc);
	loadImage(localDataDir + "/dataset/sequences/00/image_0/000001.png", img_2, currImage_lc);

#endif
	
	computeInitialPose(img_1, R_f, t_f, img_2, keyFeatures);
	
	fout << 1 << "\t";
	fout << t_f.at<double>(0) << "\t" << t_f.at<double>(1) << "\t" << t_f.at<double>(2) << "\t";
	fout << 0 << "\t" << 0 << "\n";

	// assign them to be previous
	Mat prevImage = img_2;
	vector < Point2f > prevFeatures = keyFeatures;

	Mat currImage;
	vector < Point2f > currFeatures;

	string filename;
	Mat E, R, t, mask;

	clock_t begin = clock();

	namedWindow("Road facing camera", WINDOW_AUTOSIZE); // Create a window for display.
	namedWindow("Trajectory", WINDOW_AUTOSIZE); // Create a window for display.

	Mat traj = Mat::zeros(600, 600, CV_8UC3);
	Mat trajTruth = Mat::zeros(600, 600, CV_8UC3);
	
	string fileFolder = localDataDir + "/dataset/sequences/00/"; 
	Mat R_f_left, t_f_left;
	previous_img_left = img_2;

	vector<Point2f> previous_feature_left = keyFeatures;;
	vector<Point2f> current_feature_left;

	Mat R_f_right, t_f_right;
	vector<Point2f> previous_feature_right, current_feature_right;

#ifdef REAL_TIME
	// new frame from left camera
	previous_img_left = img_2;
    getImage(0, current_img_left, leftEdge);

	// new frame from right camera
	getImage(1, current_img_right, rightEdge);
	
#else
	
	// read the first two iamges from left camera
	loadImage(localDataDir + "/dataset/sequences/00/image_0/000000.png", previous_img_left, currImage_lc );
	loadImage(localDataDir + "/dataset/sequences/00/image_0/000001.png", current_img_left,  currImage_lc);

	// read the first two iamges from right camera
	loadImage(localDataDir + "/dataset/sequences/00/image_1/000001.png", previous_img_right, currImage_rc);
	loadImage(localDataDir + "/dataset/sequences/00/image_1/000001.png", current_img_right, currImage_rc);
	
#endif
	computeInitialStereoPose(previous_img_left,
		current_img_left,
		R_f_left, 
		t_f_left, 
		previous_feature_left,
		previous_img_right,
		current_img_right,
		R_f_right, 
		t_f_right, 
		previous_feature_right);

	for (int numFrame = 2; numFrame < MAX_FRAME; numFrame++) {
		filename = combineName(localDataDir + "/dataset/sequences/00/image_0/", numFrame);
		//cout << "filename is " << filename;
		currImage_lc = imread(filename);
		getline(infile, line);
		getPosition(line, truthPosition);

#if 0
		//scale = getAbsoluteScale(numFrame, 0, t_f.at<double>(2));
		//cout << "scale is " << scale << endl;

		updatePose(filename, 
			prevImage, 
			prevFeatures, 
			currFeatures, 
			R_f, t_f);

#else
#ifdef REAL_TIME

	   // Mat leftFrame;
		getImage(0, current_img_left, leftEdge);
		
      // Mat rightFrame;
		getImage(1, current_img_right, rightEdge);
		
#else
		
		string filename1 =  combineName(localDataDir + "/dataset/sequences/00/image_0/", numFrame);
		string filename2 =  combineName(localDataDir + "/dataset/sequences/00/image_1/", numFrame);
		
		loadImage(filename1, current_img_left, currImage_lc);
		loadImage(filename2, current_img_right, currImage_rc);
		
#endif

		stereoVision(current_img_left,
			current_img_right,
			currImage_lc, 
			currImage_rc,
			previous_img_left,  previous_feature_left,  current_feature_left, 
			previous_img_right, previous_feature_right, current_feature_right,
			R_f, t_f); 
#endif

		// for plotting purpose
		double x1 = t_f.at<double>(0);
		double y1 = t_f.at<double>(1);
		double z1 = t_f.at<double>(2);

		int x = int(x1) + 300;
		int y = int(z1) + 100;

		int xTruth = int(truthPosition[0])+ 300;
		int yTruth = int(truthPosition[2])+ 100;
		// current point
		circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);
		circle(traj, Point(xTruth, yTruth), 1, CV_RGB(0, 0, 255), 2);

		rectangle(traj, Point(10,30), Point(550, 50), PLOT_COLOR, CV_FILLED);

		sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", x1, y1, z1);
		putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255),	thickness, 8);

		// Save the result
		fout << numFrame << "\t";
		fout << x1 << "\t" << y1 << "\t" << z1 << "\t" << x << "\t" << y << "\n";

		cout << "vo: x = " << PL<< x1 << "\t y = " << PL<< y1 << "\t z = " << PL<< z1 << endl;
		cout << "tr: x = " << PL<< truthPosition[0] << "\t y = " << PL<< truthPosition[1] << "\t z = " << PL<< truthPosition[2] << endl;
		imshow("Road facing camera", currImage_lc);
		imshow("Trajectory", traj);
		//imshow("Trajectory", trajTruth);

		waitKey(1);

	}

	imwrite(localDataDir + "/stereo-vo/src/final_map.png", traj);

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Total time taken: " << elapsed_secs << "s" << endl;

	return 0;
}
