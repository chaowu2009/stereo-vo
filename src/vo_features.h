#include "opencv2/opencv_modules.hpp"
#include <stdio.h>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "opencv2/xfeatures2d.hpp"

#include <iostream>
#include <ctype.h>
#include <algorithm> 
#include <iterator> 
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <cv.h>
#include <stdarg.h>
#include <iomanip>

#if defined(__linux__)
#  include <unistd.h>
#elif defined(_WIN32)
#  include <windows.h>
#  define sleep(s) Sleep((s)*1000)
#endif

using namespace cv;
using namespace std;

using namespace cv::xfeatures2d;

//#define LOGITEC
//#define KITTK
#define ARC

#ifdef LOGITEC
// new Logitec Camera
const double focal = 837.69737925956247;
const cv::Point2d pp(332.96486550136854, 220.37986827273829);
#endif

#ifdef KITTK
//  kittk camera
const double focal = 718.8560;
const cv::Point2d pp(607.1928, 185.2157);
#endif

#ifdef ARC
// 640x480 resolution
const double focal = 483.45;
const cv::Point2d pp(300.98, 253.10);
#endif

#define MIN_NUM_FEAT 2000

extern const double focal;
extern const cv::Point2d pp;

double getAbsoluteScale(int frame_id, int sequence_id, double z_cal) {

    string line;
    int i = 0;
    ifstream myfile("D:/vision/dataset/sequences/00/calib.txt");
    double x = 0, y = 0, z = 0;
    double x_prev, y_prev, z_prev;
    if (myfile.is_open())
    {
        while ((getline(myfile, line)) && (i <= frame_id))
        {
            z_prev = z;
            x_prev = x;
            y_prev = y;
            std::istringstream in(line);
            cout << line << '\n';
            for (int j = 0; j < 12; j++) {
                in >> z;
                if (j == 7) y = z;
                if (j == 3)  x = z;
            }

            i++;
        }
        myfile.close();
    }

    else {
        cout << "Unable to open file";
        return 0;
    }

    return sqrt((x - x_prev)*(x - x_prev) + (y - y_prev)*(y - y_prev) + (z - z_prev)*(z - z_prev));

}


void getPosition_Orientation(string line, float vout[3], Mat &Rout) {
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

    Rout.at<double>(0, 0) = v[0];
    Rout.at<double>(0, 1) = v[1];
    Rout.at<double>(0, 2) = v[2];
    Rout.at<double>(1, 0) = v[4];
    Rout.at<double>(1, 1) = v[5];
    Rout.at<double>(1, 2) = v[6];
    Rout.at<double>(2, 0) = v[8];
    Rout.at<double>(2, 1) = v[9];
    Rout.at<double>(2, 2) = v[10];

}

void getPosition(string line, float vout[3]) {
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



void featureTracking(Mat prevImg, Mat nextImg, vector<Point2f>& prevPts, vector<Point2f>& nextPts, vector<uchar>& status) {

    //this function automatically gets rid of points for which tracking fails

    vector<float> err;
    Size winSize = Size(21, 21);
    TermCriteria termcrit = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);

    calcOpticalFlowPyrLK(prevImg, nextImg, prevPts, nextPts, status, err, winSize, 3, termcrit, 0, 0.001);

    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for (int i = 0; i < status.size(); i++)
    {
        Point2f pt = nextPts.at(i - indexCorrection);
        if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
            if ((pt.x < 0) || (pt.y < 0)) {
                status.at(i) = 0;
            }
            prevPts.erase(prevPts.begin() + (i - indexCorrection));
            nextPts.erase(nextPts.begin() + (i - indexCorrection));
            indexCorrection++;
        }

    }

}

//uses FAST as of now, modify parameters as necessary
void featureDetection(Mat img_1, vector<Point2f>& keyFeatures) {
    vector < KeyPoint > keypoints_1;
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
    KeyPoint::convert(keypoints_1, keyFeatures, vector<int>());
}

void computeInitialPose(Mat &img_1,
    Mat &R_f,
    Mat &t_f,
    Mat &img_2,
    vector<Point2f> &points2) {

    // feature detection, tracking
    vector < Point2f > points1; //vectors to store the coordinates of the feature points
    featureDetection(img_1, points1);        //detect features in img_1
    vector < uchar > status;
    featureTracking(img_1, img_2, points1, points2, status); //track those features to img_2

    Mat E, R, t, mask;
    E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, points2, points1, R, t, focal, pp, mask);

    R_f = R.clone();
    t_f = t.clone();

}

void computeInitialStereoPose(Mat &previous_img_left,
    Mat &current_img_left,
    Mat &R_f_left,
    Mat &t_f_left,
    vector<Point2f> &points_left,
    Mat &previous_img_right,
    Mat &current_img_right,
    Mat &R_f_right,
    Mat &t_f_right,
    vector<Point2f> &points_right) {

    // feature detection, tracking
    vector < Point2f > points1; //vectors to store the coordinates of the feature points
    featureDetection(previous_img_left, points1);        //detect features in img_1
    vector < uchar > status;
    featureTracking(previous_img_left, current_img_left, points1, points_left, status); //track those features to img_2

    //recovering the pose and the essential matrix
    Mat E, R, t, mask;
    E = findEssentialMat(points_left, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, points_left, points1, R, t, focal, pp, mask);

    R_f_left = R.clone();
    t_f_left = t.clone();

    // right camera
     // feature detection, tracking
    featureDetection(previous_img_right, points1);        //detect features in img_1
    featureTracking(previous_img_right, current_img_right, points1, points_right, status); //track those features to img_2

    //recovering the pose and the essential matrix
    E = findEssentialMat(points_right, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, points_right, points1, R, t, focal, pp, mask);

    R_f_right = R.clone();
    t_f_right = t.clone();

}


void updatePose(string filename,
    Mat &prevImage,
    vector<Point2f> &prevFeatures,
    vector<Point2f> &currFeatures,
    Mat &R_f,
    Mat &t_f)
{
    cout << "entering pose estimation loop " << endl;
    Mat currImage_c = imread(filename);
    Mat currImage;
    //cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
    currImage = currImage_c;
    vector < uchar > status;
    featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);

    Mat E, R, t, mask;
    E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);

    Mat prevPts(2, prevFeatures.size(), CV_64F);
    Mat currPts(2, currFeatures.size(), CV_64F);

    //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
    //for (int i = 0; i < prevFeatures.size(); i++) {
    //	prevPts.at<double>(0, i) = prevFeatures.at(i).x;
    //	prevPts.at<double>(1, i) = prevFeatures.at(i).y;

    //	currPts.at<double>(0, i) = currFeatures.at(i).x;
    //	currPts.at<double>(1, i) = currFeatures.at(i).y;
    //}

    //scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));

    //cout << "Scale is " << scale << endl;
    double scale = 0.90;

    if ((scale > 0.1) && (t.at<double>(2) > t.at<double>(0))
        && (t.at<double>(2) > t.at<double>(1))) {
        cout << "computing pose" << endl;
        t_f = t_f + scale * (R_f * t);
        R_f = R * R_f;

    }

    // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
    if (prevFeatures.size() < MIN_NUM_FEAT) {
        featureDetection(prevImage, prevFeatures);
        featureTracking(currImage, prevImage, prevFeatures, currFeatures, status);

    }

    prevImage = currImage.clone();
    prevFeatures = currFeatures;
}

void stereoVisionIntialPose(Mat &img_left, vector<Point2f> &feature_left,
    Mat &img_right, vector<Point2f> &feature_right) {

    // left camera
    Mat R_f_left, t_f_left;
    computeInitialPose(img_left, R_f_left, t_f_left, img_left, feature_left);

    // right camera
    Mat R_f_right, t_f_right;
    computeInitialPose(img_right, R_f_right, t_f_right, img_right,
        feature_right);
}

void stereoVision(Mat &previous_img_left,
    Mat &current_img_left,
    Mat &previous_img_right,
    Mat & current_img_right,
    vector<Point2f> &previous_feature_left,
    vector<Point2f> &current_feature_left,
    vector<Point2f> &previous_feature_right,
    vector<Point2f> &current_feature_right,
    Mat &R_f,
    Mat &t_f,
    Mat &dcm)
{
    double scale = 1.0;

    // left camera feature tracking
    vector < uchar > status;
    featureTracking(previous_img_left, current_img_left, previous_feature_left, current_feature_left, status);

    // left pose estimation
    Mat E1, R1, t1, mask1;
    E1 = findEssentialMat(current_feature_left, previous_feature_left, focal, pp, RANSAC, 0.999, 1.0, mask1);
    recoverPose(E1, current_feature_left, previous_feature_left, R1, t1, focal, pp, mask1);


    // right camera feature tracking
    featureTracking(previous_img_right, current_img_right, previous_feature_right, current_feature_right, status);
    // right pose estimation
    Mat E2, R2, t2, mask2;
    E2 = findEssentialMat(current_feature_right, previous_feature_right, focal, pp, RANSAC, 0.999, 1.0, mask2);
    recoverPose(E2, current_feature_right, previous_feature_right, R2, t2, focal, pp, mask2);

    // fuse left and right pose

    Mat t = (t1 + t2) / 2.0;
    Mat R = R2;

    Mat detlaR = R2.inv()*R1;
    // normalize R
    double sum = 0;
    int cols = R.cols, rows = R.rows;

    // enlarge the diagnoal term
    for (int i = 0; i < rows; i++) {
        double * Mp = detlaR.ptr<double>(i);
        for (int j = 0; j < cols; j++) {
            if (i == i) {
                Mp[j] = Mp[i] * 2;
            }
        }
    }

    for (int i = 0; i < rows; i++) {
        const double * Mp = detlaR.ptr<double>(i);
        for (int j = 0; j < cols; j++) {
            sum += Mp[j] * Mp[j];
        }
    }
    double sqrt_of_sum = sqrt(sum);

    // Normalize
    for (int i = 0; i < rows; i++) {
        double * Mp = detlaR.ptr<double>(i);
        for (int j = 0; j < cols; j++) {
            Mp[j] = Mp[j] / sqrt_of_sum;
        }
    }

    //R = dcm;          //use the IMU DCM
    //R= R1 * detlaR;   // use corrected DCM

     //if ( (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1))) {
         //R_f = dcm;
    t_f = t_f + scale * (R_f * t);
    R_f = R * R_f;
    //}

    // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
    if (previous_feature_left.size() < MIN_NUM_FEAT) {
        featureDetection(previous_img_left, previous_feature_left);
        featureTracking(previous_img_left, current_img_left, previous_feature_left, current_feature_left, status);
    }

    previous_img_left = current_img_left.clone();
    previous_feature_left = current_feature_left;

    // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
    if (previous_feature_right.size() < MIN_NUM_FEAT) {
        featureDetection(previous_img_right, previous_feature_right);
        featureTracking(previous_img_right, current_img_right, previous_feature_right, current_feature_right, status);
    }

    previous_img_right = current_img_right.clone();
    previous_feature_right = current_feature_right;

}

void stereoPoseUpdate(Mat &R_f, Mat &t_f, vector<Point2f> &previous_feature,
    vector<Point2f> &current_feature) {

    double scale = 1.0;

    Mat E, R, t, mask;
    Mat t_f_left, R_f_left;

    E = findEssentialMat(current_feature, previous_feature, focal, pp, RANSAC,
        0.999, 1.0, mask);
    recoverPose(E, current_feature, previous_feature, R, t, focal, pp, mask);

}

// check features are matched or not between left and right.
void MatchFeatures(Mat &img_1, Mat &img_2, Mat &R_f, Mat &t_f) {

    // ref: http://stackoverflow.com/questions/27533203/how-do-i-use-sift-in-opencv-3-0-with-c
    //-- Step 1: Detect the keypoints using SURF Detector
    //-- Step 1: Detect the keypoints using SURF Detector
    // detecting keypoints
    //SurfFeatureDetector detector(400);
    Ptr < SURF > detector = SURF::create(400);
    vector<KeyPoint> keypoints1, keypoints2;
    detector->detect(img_1, keypoints1);
    detector->detect(img_2, keypoints2);

    // computing descriptors
    //SurfDescriptorExtractor extractor;
    Ptr < SURF > extractor = SURF::create();
    Mat descriptors1, descriptors2;
    extractor->compute(img_1, keypoints1, descriptors1);
    extractor->compute(img_2, keypoints2, descriptors2);

    // matching descriptors
    BFMatcher matcher;
    std::vector < DMatch > matches;
    matcher.match(descriptors1, descriptors2, matches);

    // drawing the results
    namedWindow("matches", 1);
    Mat img_matches;
    drawMatches(img_1, keypoints1, img_2, keypoints2, matches, img_matches);
    imshow("matches", img_matches);
    waitKey(0);

}


void getImage(VideoCapture &capture, Mat &imgOut, Mat &edges)
{
    if (!capture.isOpened()) {
        cout << "camera is not loaded correctly" << endl;
    }

    Mat currentFrame;
    capture >> currentFrame; // get a new frame from camera
    cvtColor(currentFrame, imgOut, COLOR_BGR2GRAY);

    // filter image
    //Mat imgTemp;
    //GaussianBlur(imgOut, imgTemp, Size(7,7), 1.5, 1.5);
    //Canny(imgTemp, edges, 0, 30, 3);
}

void loadImage(string fileName, Mat &imgOut, Mat &img_1_c) {

    //read the image
    //cout << "image name = " << fileName << endl;
    img_1_c = imread(fileName);

    if (!img_1_c.data) {
        std::cout << " --(!) Error reading images " << std::endl;
    }

    // we work with grayscale images
    cvtColor(img_1_c, imgOut, COLOR_BGR2GRAY);
}

void loadImage(string fileName, Mat &imgOut) {

    //read the image
    cout << "image name = " << fileName << endl;
    Mat img_1_c = imread(fileName);

    if (!img_1_c.data) {
        std::cout << " --(!) Error reading images " << std::endl;
    }

    // we work with grayscale images
    imgOut = img_1_c;
    //cvtColor(img_1_c, imgOut, COLOR_BGR2GRAY);
}

string combineName(string localDataDir, int numFrame, string imgFormat) {

    std::ostringstream ostr;

    ostr << std::setfill('0') << std::setw(6) << numFrame;

    string fileName = localDataDir + ostr.str() + imgFormat;

    return fileName;

}

void rectifyStereoImage(Mat &img1,
    Mat &img2,
    Mat &imgR1,
    Mat &imgR2) {

    Mat R1, R2, P1, P2, Q;
    Mat K1, K2, R;
    Vec3d T;
    Mat D1, D2;
    //  char *calib_file ="cam_stereo.yml";

      //cv::FileStorage fs1("cam_stereo.yml", cv::FileStorage::READ);
#ifdef __linux__ 
    cv::FileStorage fs1("/home/hillcrest/project/stereo-vo/src/ARC_cam_stereo.yml", cv::FileStorage::READ);
#else
    cv::FileStorage fs1("d:/vision/stereo-vo/src/ARC_cam_stereo.yml", cv::FileStorage::READ);
#endif

    if (!fs1.isOpened()) { cout << "unable to open yml file" << endl; }
    fs1["K1"] >> K1;
    fs1["K2"] >> K2;
    fs1["D1"] >> D1;
    fs1["D2"] >> D2;
    fs1["R"] >> R;
    fs1["T"] >> T;

    fs1["R1"] >> R1;
    fs1["R2"] >> R2;
    fs1["P1"] >> P1;
    fs1["P2"] >> P2;
    fs1["Q"] >> Q;

    //cout << "Q = " << Q << endl;

    cv::Mat lmapx, lmapy, rmapx, rmapy;

    cv::initUndistortRectifyMap(K1, D1, R1, P1, img1.size(), CV_32F, lmapx, lmapy);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, img2.size(), CV_32F, rmapx, rmapy);
    cv::remap(img1, imgR1, lmapx, lmapy, cv::INTER_LINEAR);
    cv::remap(img2, imgR2, rmapx, rmapy, cv::INTER_LINEAR);

}

void loadCameraParameters(Mat &K, Mat &D) {

    Mat K1, D1;
#ifdef __linux__ 
    cv::FileStorage fs1("/home/hillcrest/project/stereo-vo/src/cam_left.yml", cv::FileStorage::READ);
#else
    cv::FileStorage fs1("d:/vision/stereo-vo/src/cam_left.yml", cv::FileStorage::READ);
#endif

    if (!fs1.isOpened()) { cout << "unable to open yml file" << endl; }
    fs1["K"] >> K1;
    fs1["D"] >> D1;

    K = K1;
    D = D1;

}

void rectifyImage(Mat &imgIn,
    Mat &imgOut)
{

    Mat K;   // cameraMatrix
    Mat D;   // distCoeffs, 5

#ifdef __linux__ 
    cv::FileStorage fs1("/home/hillcrest/project/stereo-vo/src/cam_left.yml", cv::FileStorage::READ);
#else
    cv::FileStorage fs1("d:/vision/stereo-vo/src/cam_left.yml", cv::FileStorage::READ);
#endif

    if (!fs1.isOpened()) { cout << "unable to open yml file" << endl; }
    fs1["K"] >> K;
    fs1["D"] >> D;
    //imshow("original image", imgIn);

    cv::undistort(imgIn, imgOut, K, D);

    // imshow("corrected image", imgOut);
    // waitKey(1);
}

void process(Mat &pre_img_left,
    Mat &curr_img_left,
    Mat &pre_img_right,
    Mat &curr_img_right,
    Mat &R, Mat &t) {

    // ref http://avisingh599.github.io/vision/visual-odometry-full/
    // capture two consecutive images from both left and right cameras

    // undistort( compensating for lens distortion), rectify the above images(Then all the epipolar lines become parallel to the horizontal).

    // compute the disparity map D(t) from pre_img_left and pre_img_right
    //                           D(t+1) from curr_img_left and curr_img_right
    //-- 2. Call the constructor for StereoBM
    int ndisparities = 16 * 5;   /**< Range of disparity */
    int SADWindowSize = 21; /**< Size of the block window. Must be odd */

    Ptr<StereoBM> sbm = StereoBM::create(ndisparities, SADWindowSize);

    Mat Dt_previous;
    sbm->compute(pre_img_left, pre_img_right, Dt_previous);
    Mat Dt_current;
    sbm->compute(curr_img_left, curr_img_right, Dt_current);

    // use FAST algorith to detect featurs in pre_img_left and curr_img_left and match them


    // use the disparity maps D(t) and D(t+1) to calculate the 3D positions of the features detected in the previous stpes.
    // Two point clouds W(t) and W(t+1) will be obtained.

    // Select a subset of points from the above point cloud such that all the matches are mutually compatible

    // Estimate R, t from the inliners that were detected in the previous step.


}

void featureRotation(Mat &featureIn, Mat &featureOut) {
    float c_x = 800.0f, c_y = 600.0f;
    float f = 120.0f; // focal length of the first camera
    float T_x = 0.5; // the x-coordinate of the right camera with respect to the first camera( in meters)

    Mat M = (Mat_<double>(4, 4) << 1, 0, 0, -c_x, 0, 1, 0, -c_y, 0, 0, 0 - f, 0, 0 - 1 / T_x, 0);

    featureOut = M * featureIn;

}


void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt)
{

    KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter

    cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));       // set process noise
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-4));   // set measurement noise
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));             // error covariance


/** DYNAMIC MODEL **/

//  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
//  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
//  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
//  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
//  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
//  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
//  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
//  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
//  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
//  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
//  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
 //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
//  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
//  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
//  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]

                                                        // position
    KF.transitionMatrix.at<double>(0, 3) = dt;
    KF.transitionMatrix.at<double>(1, 4) = dt;
    KF.transitionMatrix.at<double>(2, 5) = dt;
    KF.transitionMatrix.at<double>(3, 6) = dt;
    KF.transitionMatrix.at<double>(4, 7) = dt;
    KF.transitionMatrix.at<double>(5, 8) = dt;
    KF.transitionMatrix.at<double>(0, 6) = 0.5*pow(dt, 2);
    KF.transitionMatrix.at<double>(1, 7) = 0.5*pow(dt, 2);
    KF.transitionMatrix.at<double>(2, 8) = 0.5*pow(dt, 2);

    // orientation
    KF.transitionMatrix.at<double>(9, 12) = dt;
    KF.transitionMatrix.at<double>(10, 13) = dt;
    KF.transitionMatrix.at<double>(11, 14) = dt;
    KF.transitionMatrix.at<double>(12, 15) = dt;
    KF.transitionMatrix.at<double>(13, 16) = dt;
    KF.transitionMatrix.at<double>(14, 17) = dt;
    KF.transitionMatrix.at<double>(9, 15) = 0.5*pow(dt, 2);
    KF.transitionMatrix.at<double>(10, 16) = 0.5*pow(dt, 2);
    KF.transitionMatrix.at<double>(11, 17) = 0.5*pow(dt, 2);


    /** MEASUREMENT MODEL **/

    //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
    //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]

    KF.measurementMatrix.at<double>(0, 0) = 1;  // x
    KF.measurementMatrix.at<double>(1, 1) = 1;  // y
    KF.measurementMatrix.at<double>(2, 2) = 1;  // z
    KF.measurementMatrix.at<double>(3, 9) = 1;  // roll
    KF.measurementMatrix.at<double>(4, 10) = 1; // pitch
    KF.measurementMatrix.at<double>(5, 11) = 1; // yaw

}

bool areTheSameImage( Mat preImage, Mat currentImage) {

    cv::Mat diffImage;
    cv::absdiff(preImage, currentImage, diffImage);

    cv::Mat foregroundMask = cv::Mat::zeros(diffImage.rows, diffImage.cols, CV_8UC1);

    float threshold = 20000;
    float dist=0;

    for(int j=0; j<diffImage.rows; ++j)
        for(int i=0; i<diffImage.cols; ++i)
        {
            cv::Vec3b pix = diffImage.at<cv::Vec3b>(j,i);

            dist = (pix[0]*pix[0] + pix[1]*pix[1] + pix[2]*pix[2]);
            dist += dist;
          
        }
 
 //   cout << "dist = " << dist << endl; 
    if(dist>threshold) {
      return false;
    }
   return true;

}
