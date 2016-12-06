This is an OpenCV 3.0 based implementation of a stereo visual odometry algorithm.

##Algorithm
1) Refer to http://avisingh599.github.io/vision/monocular-vo/. This is how this started for a monocular visual odometry.

2) Visual Stereo Odemetry for Indoor Positioning, by Fredrik Johansson
   Page 7, algorithm overview


## OpenCV version
1) OpenCV-contrib-3.1.0 (assign OPENCV_EXTRA_MODULES_PATH=your_OpenCV-contrib-3.1.0 in OpenCV cmake-gui)
2) OpenCV-3.1.0


##How to compile?
Provided with this repo is a CMakeLists.txt file, which you can use to directly compile the code as follows:
```bash
mkdir build
cd build
cmake ..
make
```

##How to run? 
After compilation, in the build directly, type the following:
```bash
./vo
```
##Before you run
In order to run this algorithm, you need to have either your own data, 
or else the sequences from [KITTI's Visual Odometry Dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php).
In order to run this algorithm on your own data, you must modify the intrinsic calibration parameters in the code.

##Contact
For any queries, contact: chaowu2009@gmail.com

##License
MIT# stereo-vo
