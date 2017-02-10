#include "opencv2/opencv_modules.hpp"
#include <stdio.h>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "opencv2/xfeatures2d.hpp"

void q2Dcm(const float q[4], cv::Mat &dcm) {
	float q00, q01, q02, q03, q11, q12, q13, q22, q23, q33;
	float w, x, y, z;
	w = q[0];
	x = q[1];
	y = q[2];
	z = q[3];
	q00 = w*w;
	q01 = w* x;
	q02 = w * y;
	q03 = w* z;
	q11 = x*x;
	q12 = x* y;
	q13 = x* z;
	q22 = y*y;
	q23 = y* z;
	q33 = z*z;

	float DCM[3][3];
	DCM[0][0] = q00 + q11 - q22 - q33;
	DCM[1][0] = (q12 - q03)*2.0f;
	DCM[2][0] = (q13 + q02) *2.0f;

	DCM[0][1] = (q12 + q03)*2.0f;
	DCM[1][1]  = q00 - q11 + q22 - q33;
	DCM[2][1] = (q23 - q01)*2.0f;

	DCM[0][2] = (q13 - q02 )* 2.0f;
	DCM[1][2] = (q23 + q01)*2.0f;
	DCM[2][2] = q00 - q11 - q22 + q33;
	//cout << "DCM[0][0] = " << DCM[0][0] << endl;
	dcm = (cv::Mat_<float>(3, 3) << DCM[0][0], DCM[0][1], DCM[0][2], DCM[1][0], DCM[1][1], DCM[1][2], DCM[2][0], DCM[2][1], DCM[2][2]);
	//cout << "dcm = " << dcm << endl;
}

 void dcm2q(const cv::Mat dcm, float q[4]) {
	 float wSq4, xSq4, ySq4, zSq4; // sometimes <0
	 float DCM[3][3];
	 for (int i = 0; i < 3; i++)
		 for (int j = 0; j < 3; j++) {
			 DCM[i][j] = dcm.at<float>(i, j);
		 }
	 
	 wSq4 = 1 + DCM[0][0] + DCM[1][1] + DCM[2][2];
	 xSq4 = 1 + DCM[0][0] - DCM[1][1] - DCM[2][2];
	 ySq4 = 1 - DCM[0][0] + DCM[1][1] - DCM[2][2];
	 zSq4 = 1 - DCM[0][0] - DCM[1][1] + DCM[2][2];

	 float w2, x2, y2, z2;
	 if ((wSq4 >= xSq4) && (wSq4 >= ySq4) && (wSq4 >= zSq4)) { 
	     // wSq4 is the largest
		 w2 = sqrt(wSq4);
		 x2 = (DCM[2][1] - DCM[1][2]) / w2;
		 y2 = -(DCM[2][0] - DCM[0][2]) / w2;
		 z2 = (DCM[1][0] - DCM[0][1]) / w2;

	 }
	 else if ((xSq4 >= ySq4) && (xSq4 >= zSq4)) { 
	     // xSq4 is the largest
		 x2 = sqrt(xSq4);
		 w2 = (DCM[2][1] - DCM[1][2]) / x2;
		 y2 = (DCM[1][0] + DCM[0][1]) / x2;
		 z2 = (DCM[2][0] + DCM[0][2]) / x2;

	 }
	 else if (ySq4 >= zSq4) { // ySq4 is the largest
		 y2 = sqrt(ySq4);
		 w2 = -(DCM[2][0] - DCM[0][2]) / y2;
		 x2 = (DCM[1][0] + DCM[0][1]) / y2;
		 z2 = (DCM[2][1] + DCM[1][2]) / y2;

	 }
	 else { // zSq4 is the largest
		 z2 = sqrt(zSq4);
		 w2 = (DCM[1][0] - DCM[0][1]) / z2;
		 x2 = (DCM[2][0] + DCM[0][2]) / z2;
		 y2 = (DCM[2][1] + DCM[1][2]) / z2;
	 }

	 q[0] = w2 / 2.0f;
	 q[1] = x2 / 2.0f;
	 q[2] = y2 / 2.0f;
	 q[3] = z2/ 2.0f;
 }

void qmult(const float q1[4], const float q2[4], float qOut[4]){

    qOut[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    qOut[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2]; 
    qOut[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1]; 
    qOut[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];  

}


void qConj(const float qIn[4], float qOut[4]){

    for(int i = 1; i<4; i++){
         qOut [i ] = -qIn[i];
    }

    qOut[0] = qIn[0];

}

void qNorm(float qIn[4], float qOut[4]){

    float norm= 0;
    for(int i = 0; i<4; i++){
         norm += qIn[i]*qIn[i];
         qOut[i] = qIn[i];
    }

    norm = sqrt(norm);

    for(int i = 0; i<4; i++){
         qOut[i] /= norm;
}

}


void qvrot(const float qIn[4], const float vIn[3], float vOut[3]){

    float vAugmented[4], q2[4];
    vAugmented[0] = 0;
    for(int i = 0; i<3; i++){
        vAugmented[i+1] = vIn[i];
    }

    float qTemp[4], qconj[4];
    qConj(qIn, qconj);

    qmult(qIn, vAugmented, qTemp);

    qmult(qTemp, qconj, q2);
    for(int i = 1; i<3; i++){
        vOut[i-1] = q2[i];
    }

}

void slerp(float q1[4], float q2[4], float w, float qOut[4]){

     for(int i =0 ; i<4; i++){
         qOut[i] = (1-w)*q1[i] + w* q2[i];     
     }

     float temp[4];
     qNorm(qOut, temp);
     for(int i =0 ; i<4; i++){
         qOut[i] = temp[i];
     }

}
