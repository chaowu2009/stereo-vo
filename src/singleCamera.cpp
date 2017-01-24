#include "rotation.h"
#include "vo_features.h"
#include <iomanip>
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

//#define REAL_TIME 1
//#define SHOW_IMAGE_ONLY 1
//#define BNO

// new camera
const double focal = 837.69737925956247;
const cv::Point2d pp (332.96486550136854, 220.37986827273829);
//  kittk camera
//const double focal = 718.8560;
//const cv::Point2d pp(607.1928, 185.2157);


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
	dcm = (cv::Mat_<float>(3, 3) << 1.0, 0.0, 0.0, 0.0,1.0,0.0, 0.0, 0.0, 1.0);
	
#ifdef __linux__ 
    int fd = initBNO();
    
    readQuaternion(fd, q);  //the first sample is bad. So clean the buffer
#endif

#ifdef __linux__ 
    //linux code goes here

    std::string imgDir="/home/cwu/project/dataset/images/4";
    std::string imgFormat = ".jpg";
    std::string timeStampFile = imgDir + "timeStamp.txt";
    std::string resultFile = imgDir + "vo_result.txt";
 
    MAX_FRAME = 1000;
    
	std::string quaternionFile = imgDir + "q.txt";
	std::fstream infile(quaternionFile.c_str());
	std::string line;
#else
    // windows code goes here
  //  string localDataDir = "d:/vision";
    string resultFile   = "d:/vision/stereo-vo/src/vo_result.txt";
    string imgDir       = "d:/vision/dataset/images/3/";
   
	std::string quaternionFile = imgDir + "q.txt";
	std::fstream infile(quaternionFile);
	std::string line;
#endif

    Mat current_img_left, current_img_right;

    Mat leftEdge;

    // for plotting purpose
    Mat currImage_lc; 

    Mat R_f, t_f; //the final rotation and translation vectors

    Mat img_1, img_2;  // two consecutive images from the same camera

#ifdef REAL_TIME
    cout << "Running at real-time" <<endl;
	std::ofstream fpTimeStamp;
	fpTimeStamp.open(timeStampFile.c_str());

    VideoCapture left_capture(LEFT);
    left_capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    left_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);

    left_capture.read(img_1);
    begin = clock();

#else
    cout << "Running at simulation mode!" << endl;
#endif

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

#else
    loadImage(imgDir + "/img_left/1" + imgFormat, img_1, currImage_lc);

#endif

    computeInitialPose(img_1, R_f, t_f, img_2, keyFeatures);

    // assign them to be previous
    Mat prevImage = img_2;
    vector < Point2f > prevFeatures = keyFeatures;

    Mat currImage;
    vector < Point2f > currFeatures = keyFeatures;

    string filename;
    Mat E, R, t, mask;

    namedWindow("Trajectory", WINDOW_AUTOSIZE); // Create a window for display.

    Mat traj = Mat::zeros(640, 480, CV_8UC3);
    Mat trajTruth = Mat::zeros(640, 480, CV_8UC3);

    Mat R_f_left, t_f_left;
    Mat previous_img_left = img_2;

    vector<Point2f> previous_feature_left = keyFeatures;;
    vector<Point2f> current_feature_left;


#ifdef REAL_TIME
    // new frame from left camera
    previous_img_left = img_2;
    left_capture.read(current_img_left);

#else

    loadImage(imgDir + "/img_left/1"+ imgFormat, previous_img_left, currImage_lc );
    loadImage(imgDir + "/img_left/2" +imgFormat, current_img_left,  currImage_lc);

    rectifyImage(previous_img_left, previous_img_left);

#endif
    computeInitialPose(previous_img_left,
        current_img_left,
        R_f_left, 
        t_f_left, 
        previous_feature_left);

//	R_f_left = dcm;
	
    for (int numFrame = 2; numFrame < MAX_FRAME; numFrame++) {
        cout <<"numFrame = " << numFrame << endl;
#ifdef REAL_TIME

        // Mat leftFrame;
        bool readSuccess1 = left_capture.read(current_img_left);
        if (readSuccess1){ currImage_lc = current_img_left;}

#else

        stringstream ss;
        ss << numFrame;
        string idx = ss.str();

        string filename1 =  imgDir + "/img_left/" + idx + imgFormat;

        loadImage(filename1, current_img_left, currImage_lc);

        rectifyImage(current_img_left, current_img_left);

		std::getline(infile, line);
		std::istringstream iss(line);
		if (!(iss >> q[0] >> q[1] >> q[2] >> q[3])) { break; }
		//cout << "q=" << q[0] << " "<< q[1] << " " << q[2] << " " << q[3]  << endl;
		q2Dcm(q, dcm);
		//cout <<"dcm " << dcm <<  endl;

#endif
       
        updatePose(filename, 
            prevImage, 
            prevFeatures, 
            currFeatures, 
            R_f, 
            t_f);

        // for plotting purpose
        double x1 = t_f.at<double>(0);
        double y1 = t_f.at<double>(1);
        double z1 = t_f.at<double>(2);

        int x = int(x1) +320;
        int y = int(z1) +240;

        // output to the screen
        cout << "numFrame:" << numFrame <<" vo: x = " << PL<< x1 << "\t y = " << PL<< y1 << "\t z = " << PL<< z1 << endl;
        // current point
        circle(traj, Point(x, y), 0.2, CV_RGB(255, 0, 0), 2);
        rectangle(traj, Point(10,30), Point(500, 50), PLOT_COLOR, CV_FILLED);

        sprintf(text, "Coordinates: x = %04fm y = %04fm z = %04fm", x1, y1, z1);
        putText(traj, text, textOrg, fontFace, fontScale, Scalar::all(255),	thickness, 8);

        // plot them
        imshow("Left camera", currImage_lc);
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
