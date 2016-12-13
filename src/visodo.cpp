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

#include "vo_features.h"
#include <iomanip>
#include <fstream>

using namespace cv;
using namespace std;

#define MAX_FRAME 4541
#define MIN_NUM_FEAT 200
#define PLOT_COLOR CV_RGB(0, 0, 0)
#define PL std::setprecision(3)


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

// IMP: Change the file directories (4 places) according to where your dataset is saved before running!

double focal = 718.8560;
cv::Point2d pp(607.1928, 185.2157);

double scale = 1.00;
char text[100];
int fontFace = FONT_HERSHEY_PLAIN;
double fontScale = 1;
int thickness = 1;
cv::Point textOrg(10, 50);

int main(int argc, char** argv) {

     //obtain truth for plot comparison
	string posePath = "/home/cwu/Downloads/dataset/poses/00.txt";
    std::ifstream infile("/home/cwu/Downloads/dataset/poses/00.txt");

    std::string line; 
    float truthPosition[3] ;

    getline(infile, line);
    getPosition(line, truthPosition);

	// Open a txt file to store the results
	ofstream fout("/home/cwu/project/stereo-vo/src/vo_result.txt");
	if (!fout) {
		cout << "File not opened!" << endl;
		return 1;
	}

    
	// Set initial orientation and position aligned with left camera
	// which will be the orientation and position of the system.
	// Then right camera has an offset of 0.5 meters from left camera
	// The right camera has the same orientation as the left camera.

	Mat R_f, t_f; //the final rotation and translation vectors

	Mat img_1, img_2;
	vector < Point2f > points2;
	char folder[100];

	// use the first two images from left camera to compute the init values.
	computeInitialPose(folder, R_f, t_f, img_2, points2);
    
    fout << 1 << "\t";
    fout << t_f.at<double>(0) << "\t" << t_f.at<double>(1) << "\t" << t_f.at<double>(2) << "\t";
    fout << 0 << "\t" << 0 << "\n";
  
	Mat prevImage = img_2;
	Mat currImage;
	vector < Point2f > prevFeatures = points2;
	vector < Point2f > currFeatures;

	char filename[100];
	Mat E, R, t, mask;

	clock_t begin = clock();

	namedWindow("Road facing camera", WINDOW_AUTOSIZE); // Create a window for display.
	namedWindow("Trajectory", WINDOW_AUTOSIZE); // Create a window for display.

	Mat traj = Mat::zeros(600, 600, CV_8UC3);
	Mat trajTruth = Mat::zeros(600, 600, CV_8UC3);
	Mat currImage_c;
    Mat currImage_rc; 

    string fileFolder = "/home/cwu/Downloads/dataset/sequences/00/"; 
    Mat R_f_left, t_f_left, previous_img_left;
	vector<Point2f> previous_feature_left, current_feature_left;

	Mat R_f_right, t_f_right, previous_img_right;
	vector<Point2f> previous_feature_right, current_feature_right;

    char filename1[200], filename2[200];
	sprintf(filename1, "/home/cwu/Downloads/dataset/sequences/00/image_0/%06d.png", 0);
	sprintf(filename2, "/home/cwu/Downloads/dataset/sequences/00/image_0/%06d.png", 0);


	//read the first two frames from the dataset
	Mat current_img_left  = imread(filename1);
	Mat current_img_right = imread(filename2);
    
    previous_img_left = current_img_left;
    previous_img_right = current_img_right;

	if (!current_img_left.data || !current_img_right.data) {
	    std::cout << " --(!) Error reading images " << std::endl;
	}

	// we work with grayscale images
	cvtColor(current_img_left, current_img_left, COLOR_BGR2GRAY);
	cvtColor(current_img_right, current_img_right, COLOR_BGR2GRAY);
	//vector < Point2f > current_feature_left; //vectors to store the coordinates of the feature points
	featureDetection(current_img_left, current_feature_left);

	//vector < Point2f > current_feature_right; //vectors to store the coordinates of the feature points
	featureDetection(current_img_right, current_feature_right);

    previous_feature_left = current_feature_left;
    previous_feature_right = current_feature_right;
	//stereoVisionIntialPose(fileFolder, 
	//	                   img_left, feature_left, 
	//	                   img_right, feature_right);

	for (int numFrame = 2; numFrame < MAX_FRAME; numFrame++) {
		sprintf(filename, "/home/cwu/Downloads/dataset/sequences/00/image_0/%06d.png", numFrame);
		currImage_c = imread(filename);
		getline(infile, line);
        getPosition(line, truthPosition);

#if 1
		//scale = getAbsoluteScale(numFrame, 0, t_f.at<double>(2));
		//cout << "scale is " << scale << endl;

		updatePose(filename, 
			       prevImage, 
			       prevFeatures, 
			       currFeatures, 
			       R_f, t_f);

#else
       currImage_c = imread(filename);

       stereoVision(fileFolder,  
                    numFrame, 
                    currImage_c, 
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
		
		//rectangle(traj, Point(10,30), Point(550, 50), PLOT_COLOR, CV_FILLED);
        
		//sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", x1, y1, z1);
		//putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255),	thickness, 8);

		// Save the result
		fout << numFrame << "\t";
		fout << x1 << "\t" << y1 << "\t" << z1 << "\t" << x << "\t" << y << "\n";
        
        cout << "vo: x = " << PL<< x1 << "\t y = " << PL<< y1 << "\t z = " << PL<< z1 << endl;
        cout << "tr: x = " << PL<< truthPosition[0] << "\t y = " << PL<< truthPosition[1] << "\t z = " << PL<< truthPosition[2] << endl;
		imshow("Road facing camera", currImage_c);
		imshow("Trajectory", traj);
		//imshow("Trajectory", trajTruth);

		waitKey(1);

	}

    imwrite("/home/cwu/project/stereo-vo/src/final_map.png", traj);

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Total time taken: " << elapsed_secs << "s" << endl;

	return 0;
}
