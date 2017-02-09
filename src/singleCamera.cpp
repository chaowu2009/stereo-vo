
#include "vo_features.h"
#include "BNO080.h"
#include "iRobot.h"

using namespace cv;
using namespace std;

#define MAX_FRAME 10000

#define MIN_NUM_FEAT 100
#define LEFT 0
#define RIGHT 1

#define MIN_NUM_FEAT 2000

#define WEBCAM 0
#define LEFT 1
#define RIGHT 2


#define REAL_TIME
#define PLOT_COLOR CV_RGB(0, 0, 0)
#define PL std::setprecision(6)


int main(int argc, char** argv) {

    cv::KalmanFilter KF;
    int nStates = 18;
    int nMeasurements = 6;
    int nInputs = 0;

    int fd;//= initBNO080();

    float q[4];
    readQ(fd, q);

    //  initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);

    char text[100];
    int fontFace = FONT_HERSHEY_PLAIN;
    double fontScale = 1;
    int thickness = 1;
    cv::Point textOrg(10, 50);

    Mat K, D;
    loadCameraParameters(K, D);

    Mat img_1, img_2;
    Mat img_1_c, img_2_c;
    Mat R_f, t_f; //the final rotation and tranlation vectors containing the 

    ofstream myfile;
    myfile.open("results1_1.txt");

#ifdef REAL_TIME

    cout << "Running at real-time" << endl;

    VideoCapture left_capture(LEFT);
    left_capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    left_capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    left_capture.read(img_1_c);
    cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
    //        undistort(img_1, img_1, K, D);

    left_capture.read(img_2_c);
    cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);
    //        undistort(img_2, img_2, K, D);

#else
    double scale = 1.00;
    char filename1[200];
    char filename2[200];
    sprintf(filename1, "D:/vision/dataset/sequences/00/image_1/%06d.png", 0);
    sprintf(filename2, "D:/vision/dataset/sequences/00/image_1/%06d.png", 1);

    //read the first two frames from the dataset
    img_1_c = imread(filename1);
    img_2_c = imread(filename2);
    cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
    cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

#endif
    if (!img_1.data || !img_2.data) {
        std::cout << " --(!) Error reading images " << std::endl; return -1;
    }


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

    namedWindow("Road facing camera", WINDOW_AUTOSIZE); // Create a window for display.
    namedWindow("Trajectory", WINDOW_AUTOSIZE);         // Create a window for display.

    Mat traj = Mat::zeros(600, 600, CV_8UC3);

    for (int numFrame = 2; numFrame < MAX_FRAME; numFrame++) {
#ifdef REAL_TIME
        left_capture.read(img_1);
        // read BNO
        readQ(fd, q);
        //   undistort(img_1, img_1, K, D);
#else
        sprintf(filename, "D:/vision/dataset/sequences/00/image_1/%06d.png", numFrame);
        Mat img_1 = imread(filename);
#endif
        cvtColor(img_1, currImage, COLOR_BGR2GRAY);
        vector<uchar> status;
        featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
        int trackedFeatureNumber = 0;
        for (int m = 0; m < status.size(); m++) {
            if (status[m] > 0) { trackedFeatureNumber++; }
        }

        if (trackedFeatureNumber > 50) {
            E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
            recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
        }
        else {
            cout << "untracked and no update!" << endl;
        }


        // update pose
        float deltaT = 0.5;// / 16.0;
        t_f = t_f *deltaT + (R_f*t);
        R_f = R*R_f;

        // update featurs if trakced features go below a particular threshold
        if (prevFeatures.size() < MIN_NUM_FEAT) {
            //cout << "Number of tracked features reduced to " << prevFeatures.size() << endl;
            //cout << "trigerring redection" << endl;
            featureDetection(prevImage, prevFeatures);
            featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
        }

        prevImage = currImage.clone();
        prevFeatures = currFeatures;

        double x1 = t_f.at<double>(0);
        double y1 = t_f.at<double>(1);
        double z1 = t_f.at<double>(2);

        int x = int(x1) + 300;
        int y = int(z1) + 300;

        cout << "numFrame:" << numFrame << " mo: x =" << PL << x1 << "  y= " << PL << y1 << "  z= " << PL << z1 << endl;

        circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);

        rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0), CV_FILLED);
        sprintf(text, "Coordinates: x = %0fm y = %02fm z = %02fm", x1, y1, z1);
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
