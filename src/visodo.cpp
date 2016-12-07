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

using namespace cv;
using namespace std;

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 200

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

	// Set initial orientation and position aligned with left camera
	// which will be the orientation and position of the system.
	// Then right camera has an offset of 0.5 meters from left camera
	// The right camera has the same orientation as the left camera.

	Mat R_f, t_f; //the final rotation and tranlation vectors

	Mat img_1, img_2;
	vector < Point2f > points2;
	char folder[100];

	// use the first two images from left camear to compute the init values.
	computeInitialPose(folder, R_f, t_f, img_2, points2);

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
	Mat currImage_c;
#if 0
	for(int numFrame=2; numFrame < MAX_FRAME; numFrame++) {
		sprintf(filename, "/home/cwu/Downloads/dataset/sequences/00/image_1/%06d.png", numFrame);
		//cout << numFrame << endl;
		updatePose(filename, currImage_c, prevImage, currImage, prevFeatures, currFeatures, R_f, t_f);

		int x = int(t_f.at<double>(0)) + 300;
		int y = int(t_f.at<double>(2)) + 100;
		circle(traj, Point(x, y) ,1, CV_RGB(255,0,0), 2);

		rectangle( traj, Point(10, 30), Point(550, 50), CV_RGB(0,0,0), CV_FILLED);
		sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
		putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);

		imshow( "Road facing camera", currImage_c );
		imshow( "Trajectory", traj );

		waitKey(1);

	}

	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Total time taken: " << elapsed_secs << "s" << endl;

	//cout << R_f << endl;
	//cout << t_f << endl;
#else

	img_1 = imread(
			"/home/cwu/Downloads/dataset/sequences/00/image_0/000000.png",
			IMREAD_GRAYSCALE);
	img_2 = imread(
			"/home/cwu/Downloads/dataset/sequences/00/image_0/000000.png",
			IMREAD_GRAYSCALE);

	if (!img_1.data || !img_2.data) {
		std::cout << " --(!) Error reading images " << std::endl;
	}

	bool matched = MatchFeatures(img_1, img_2);

#endif
	return 0;
}
