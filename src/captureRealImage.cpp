#include "vo_features.h"
#include <iomanip>
#include <fstream>
#include <ostream>
#include <string>

using namespace cv;
using namespace std;


// new camera
const double focal = 837.69737925956247;
const cv::Point2d pp (332.96486550136854, 220.37986827273829);


int LEFT = 0;
int RIGHT = 1;

int main(int argc, char** argv) {

    clock_t begin = clock();
	clock_t currentFrameClock;
	float deltaTinSecond = 0.0f;
    int MAX_FRAME = 1000;
	    
    //linux code goes here
    std::string resultFile = "/home/cwu/project/stereo-vo/src/vo_result.txt";
    std::string imgDir="/home/cwu/project/dataset/images/9/";
    cout << "capturing data only, saved at " << imgDir << endl;
    std::string imgFormat = ".jpg";
    std::string timeStampFile = imgDir + "timeStamp.txt";
    MAX_FRAME = 1000;
    
	std::string quaternionFile = imgDir + "q.txt";
	std::fstream infile(quaternionFile.c_str());
	std::string line;

    Mat current_img_left, current_img_right;
    Mat previous_img_left, previous_img_right; 
    Mat leftEdge, rightEdge;

    // for plotting purpose
    Mat currImage_lc, currImage_rc; 

    Mat R_f, t_f; //the final rotation and translation vectors

    Mat img_1, img_2;  // two consecutive images from the same camera

    cout << "Running at real-time" <<endl;
	std::ofstream fpTimeStamp;
	fpTimeStamp.open(timeStampFile.c_str());

    VideoCapture left_capture(LEFT);

    VideoCapture right_capture(RIGHT);

    left_capture.read(img_1);
    begin = clock();

    int numFrame = 1;
    while (1) {
        left_capture.read(current_img_left);
        right_capture.read(current_img_right);

       // imshow("LEFT image", current_img_left);
       // imshow("RIGHT image", current_img_right);
       // waitKey(1);
        
        stringstream ss;
        ss << numFrame;
        string idx = ss.str();
        string leftImg  = imgDir + "img_left/" + idx + imgFormat;
        string rightImg = imgDir + "img_right/" + idx + imgFormat;
        //cout << "leftImg = " << leftImg << endl;
        Mat imgOut;
        cvtColor(current_img_left, imgOut, COLOR_BGR2GRAY);
        imwrite(leftImg,  current_img_left);

        cvtColor(current_img_right, imgOut, COLOR_BGR2GRAY);
        imwrite(rightImg, imgOut);
        
        currentFrameClock = clock();
        deltaTinSecond = double(currentFrameClock-begin)/CLOCKS_PER_SEC;
        fpTimeStamp << numFrame << "," << deltaTinSecond << endl;
        
        numFrame++;
  //    cout << "numFrame = " << numFrame << endl;
        if (numFrame > MAX_FRAME) {
            fpTimeStamp.close();
            cout << "capture done" << endl;
            return 0;
        }

    }
}
