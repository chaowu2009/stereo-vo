#include <opencv2/opencv.hpp>
#include <iomanip>
#include <fstream>

using namespace cv;
using namespace std;



int main(int argc, char** argv) {

    string imgFile="/home/cwu/project/dataset/images/9/img_left/1.jpg";
    Mat  src = imread(imgFile);
   if (src.empty()){
       cout << "error loading picture " << endl;
    }

    
    Point2f srcQuad[] ={
        Point2f(0,0),
        Point2f(src.cols-1,0),
        Point2f(src.cols-1,src.rows-1),
        Point2f(0,src.rows-1)
        };
        
    Point2f dstQuad[] ={
        Point2f(src.cols*0.5f, src.rows*0.33f),
        Point2f(src.cols*0.9f, src.rows*0.25f),
        Point2f(src.cols*0.8f, src.rows*0.9f),
        Point2f(src.cols*0.2f, src.rows*0.7f)
        };

     Mat warp_mat = getPerspectiveTransform(srcQuad, dstQuad);
     cout <<"warp_mat" <<warp_mat << endl;
     Mat dst;
     warpPerspective(src,dst, warp_mat, src.size(), INTER_LINEAR, BORDER_CONSTANT, Scalar());
     
     for(int i = 0; i < 4; i++){
     
        circle(dst, dstQuad[i], 5, Scalar(255,0,255), -1, 0);
    }    
    
    imshow("Perspective Transfomr Test", dst);
    waitKey();
    
    return 0;
}
