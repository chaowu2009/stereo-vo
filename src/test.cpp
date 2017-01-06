#include "vo_features.h"
#include <iomanip>
#include <fstream>

using namespace cv;
using namespace std;


#ifdef REAL_TIME
// new camera
const double focal = 837.69737925956247;
const cv::Point2d pp (332.96486550136854, 220.37986827273829);
#else
const double focal = 718.8560;
const cv::Point2d pp(607.1928, 185.2157);
#endif


int main(int argc, char** argv) {

#ifdef __linux__ 
    //linux code goes here
    string localDataDir = "/home/cwu/Downloads";
    string resultFile = "/home/cwu/project/stereo-vo/src/vo_result.txt";
    //cout << "localDataDir is " << localDataDir << endl;
    string imgDir="/home/cwu/project/dataset/images/1/";
#else
    // windows code goes here
  //  string localDataDir = "d:/vision";
    string resultFile   = "d:/vision/stereo-vo/src/vo_result.txt";
    string imgDir       = "d:/vision/dataset/images/1/";
    string localDataDir = "d:/vision/dataset//images/1/";
    //cout << "localDataDir is " << localDataDir << endl;
#endif

    Mat current_img_left, current_img_right;
    Mat rectified_img_left, rectified_img_right;

    current_img_left  = imread(localDataDir + "/img_left/2.png");
    current_img_right = imread(localDataDir + "/img_right/2.png");

    rectifyImage(current_img_left, current_img_right, rectified_img_left, rectified_img_right);

    imshow("left image before", current_img_left);
    imshow("left image after", rectified_img_left);
    imshow("right image before", current_img_right);
    imshow("right image after", rectified_img_right);
    
    return 0;
}
