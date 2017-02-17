import numpy as np
import cv2
import glob
import yaml

def calibrateSingleCamera(folder = "/home/cwu/project/stereo-calibration/calib_imgs/3/left/",
                                              calFileName = "calibration.yaml", plotFigure= True):

        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        #Number of inner corners per a chessboard row and column
        gridWidth = 6;
        gridHeight = 9;
        objp = np.zeros((gridWidth*gridHeight,3), np.float32)
        objp[:,:2] = np.mgrid[0:gridHeight,0:gridWidth].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.


        images = glob.glob(folder + '*.jpg')

        for fname in images:
            print("loading ", fname)
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (gridHeight,gridWidth),None)

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners)

                if plotFigure:
                    # Draw and display the corners
                    cv2.drawChessboardCorners(img, (gridHeight,gridWidth), corners,ret)
                    cv2.imshow('img',img)
                    cv2.waitKey(500)

        cv2.destroyAllWindows()

        ret, calibration_matrix, dist_coeff, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

        img = cv2.imread(folder+ 'left_12.jpg')
        h,  w = img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(calibration_matrix,dist_coeff,(w,h),1,(w,h))

        # undistort
        dst = cv2.undistort(img, calibration_matrix, dist_coeff, None, newcameramtx)

        # crop the image
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        cv2.imwrite('calibresult.png',dst)

        tot_error = 0
        for i in xrange(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], newcameramtx, dist_coeff)
            error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            tot_error += error
             
        print "mean error: ", tot_error/len(objpoints)

        data = {"K": [newcameramtx.tolist()], "D": [dist_coeff.tolist()]}
#        K=[[ 532.80990646 ,0.0,342.49522219],[0.0,532.93344713,233.88792491],[0.0,0.0,1.0]]
#        D = [-2.81325798e-01,2.91150014e-02,1.21234399e-03,-1.40823665e-04,1.54861424e-01]
#        data = {"K": camera_matrix, "D": dist_coeff}
        print("camera_matrix = ", data["K"])
        print("dist_coeff = ", data["D"])

        with open(calFileName, "wb") as f:
            yaml.dump(data, f)


if __name__ == "__main__":
    calibrateSingleCamera()
