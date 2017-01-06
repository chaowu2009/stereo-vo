#include "vo_features.h"
#include <iomanip>
#include <fstream>

using namespace cv;
using namespace std;

//#define MAX_FRAME 200
//#define MIN_NUM_FEAT 200
#define PLOT_COLOR CV_RGB(0, 0, 0)
#define PL std::setprecision(3)

double scale = 1.00;
char text[100];
int fontFace = FONT_HERSHEY_PLAIN;
double fontScale = 1;
int thickness = 1;
cv::Point textOrg(10, 50);

//#define REAL_TIME 1
//#define SHOW_IMAGE_ONLY 1

#ifdef REAL_TIME
// new camera
const double focal = 837.69737925956247;
const cv::Point2d pp (332.96486550136854, 220.37986827273829);
#else
const double focal = 718.8560;
const cv::Point2d pp(607.1928, 185.2157);
#endif

int LEFT = 0;
int RIGHT = 1;

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
    Mat previous_img_left, previous_img_right; 
    Mat leftEdge, rightEdge;

    // for plotting purpose
    Mat currImage_lc, currImage_rc; 

    Mat R_f, t_f; //the final rotation and translation vectors

    Mat img_1, img_2;  // two consecutive images from the same camera

#ifdef REAL_TIME
    cout << "Running at real-time" <<endl;

    VideoCapture left_capture(LEFT);
    left_capture.set(CV_CAP_PROP_FPS,100);
    left_capture.set(CV_CAP_PROP_BUFFERSIZE,3);

    VideoCapture right_capture(RIGHT);
    right_capture.set(CV_CAP_PROP_FPS,100);
    right_capture.set(CV_CAP_PROP_BUFFERSIZE,3);

    left_capture.read(img_1);


#ifdef SHOW_IMAGE_ONLY
    namedWindow("LEFT image", 0);
    namedWindow("RIGHT image", 1);

    int numFrame = 1;
    while (1) {
        left_capture.read(current_img_left);
        right_capture.read(current_img_right);

        imshow("LEFT image", current_img_left);
        imshow("RIGHT image", current_img_right);
        
        stringstream ss;
        ss << numFrame;
        string idx = ss.str();
        string leftImg  = imgDir + "img_left/" + idx + ".png";
        string rightImg = imgDir + "img_right/" + idx + ".png";
        //cout << "leftImg = " << leftImg << endl;
        Mat imgOut;
        cvtColor(current_img_left, imgOut, COLOR_BGR2GRAY);
        imwrite(leftImg,  current_img_left);

        cvtColor(current_img_right, imgOut, COLOR_BGR2GRAY);
        imwrite(rightImg, imgOut);
        numFrame++;
        if (numFrame > 300) {return 0;}
        waitKey(100); //micro second
    }

#endif

#else
    cout << "Running at simulation mode!" << endl;
#endif

    //obtain truth for plot comparison
    string posePath = localDataDir + "/dataset/poses/00.txt";
    std::ifstream infile(posePath.c_str());

    std::string line; 
    float truthPosition[3] ;
    Mat truthOrientation;

    //getline(infile, line);
    //getPosition(line, truthPosition);

    // Open a txt file to store the results
    ofstream fout(resultFile.c_str());
    if (!fout) {
        cout << "File not opened!" << endl;
        return 1;
    }

    // features
    vector < Point2f > keyFeatures;

#ifdef REAL_TIME

    //left camera, second frame
    left_capture.read(img_2);
    // right camera	
    right_capture.read(previous_img_right);

#else
    // use the first two images from left camera to compute the init values.
    //loadImage(localDataDir + "/dataset/sequences/00/image_0/000000.png", img_1, currImage_lc);
    //loadImage(localDataDir + "/dataset/sequences/00/image_0/000001.png", img_2, currImage_lc);
    loadImage(localDataDir + "/img_left/1.png", img_1, currImage_lc);
    loadImage(localDataDir + "/img_right/1.png", img_2, currImage_lc);

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

    //namedWindow("Road facing camera", WINDOW_AUTOSIZE); // Create a window for display.
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
    left_capture.read(current_img_left);

    // new frame from right camera
    right_capture.read(current_img_right);

#else

    // read the first two iamges from left camera
    //loadImage(localDataDir + "/dataset/sequences/00/image_0/000000.png", previous_img_left, currImage_lc );
    //loadImage(localDataDir + "/dataset/sequences/00/image_0/000001.png", current_img_left,  currImage_lc);

    loadImage(localDataDir + "/img_left/1.png", previous_img_left, currImage_lc );
    loadImage(localDataDir + "/img_left/2.png", current_img_left,  currImage_lc);

    // read the first two iamges from right camera
    loadImage(localDataDir + "/img_right/1.png", previous_img_right, currImage_rc);
    loadImage(localDataDir + "/img_right/2.png", current_img_right, currImage_rc);

    rectifyImage(previous_img_left, previous_img_right, previous_img_left, previous_img_right);
    rectifyImage(current_img_left, current_img_right, current_img_left, current_img_right);

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
        //filename = combineName(localDataDir + "/dataset/sequences/00/image_0/", numFrame);
        //cout << "numFrame is " << numFrame << endl;
        //getline(infile, line);
        //getPosition(line, truthPosition);

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
        bool readSuccess1 = left_capture.read(current_img_left);
        if (readSuccess1){ currImage_lc = current_img_left;}

        // Mat rightFrame;
        bool readSuccess2 = right_capture.read(current_img_right);
        if (readSuccess2) {currImage_rc = current_img_right; }

#else

        //string filename1 =  combineName(localDataDir + "/dataset/sequences/00/image_0/", numFrame);
        //string filename2 =  combineName(localDataDir + "/dataset/sequences/00/image_1/", numFrame);
        stringstream ss;
        ss << numFrame;
        string idx = ss.str();

        string filename1 =  localDataDir + "/img_left/" + idx + ".png";
        string filename2 =  localDataDir + "/img_right/" + idx + ".png";

        loadImage(filename1, current_img_left, currImage_lc);
        loadImage(filename2, current_img_right, currImage_rc);

        rectifyImage(current_img_left, current_img_right, current_img_left, current_img_right);

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

        // output to the screen
        cout << "vo: x = " << PL<< x1 << "\t y = " << PL<< y1 << "\t z = " << PL<< z1 << endl;
#ifndef REAL_TIME
        //cout << "tr: x = " << PL<< truthPosition[0] << "\t y = " << PL<< truthPosition[1] << "\t z = " << PL<< truthPosition[2] << endl;
#endif
        // current point
        circle(traj, Point(x, y), 0.2, CV_RGB(255, 0, 0), 2);
#ifndef REAL_TIME
        //circle(traj, Point(xTruth, yTruth), 0.2, CV_RGB(0, 0, 255), 2);
#endif
        //rectangle(traj, Point(10,30), Point(550, 50), PLOT_COLOR, CV_FILLED);

        sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", x1, y1, z1);
        putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255),	thickness, 8);

        // plot them
#ifndef REAL_TIME
        imshow("Road facing camera", currImage_lc);
#endif
        imshow("Trajectory", traj);
        //imshow("Trajectory", trajTruth);

        // Save the result
        fout << numFrame << "\t";
        fout << x1 << "\t" << y1 << "\t" << z1 << "\t" << x << "\t" << y << "\n";

        waitKey(2);  //microsecond
    }

    imwrite(localDataDir + "/final_map.png", traj);

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "Total time taken: " << elapsed_secs << "s" << endl;

    return 0;
}
