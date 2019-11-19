# Extended Kalman Filter Project


This project is a part of:  
 [![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)  
 and it's basing on [CarND-Extended-Kalman-Filter-Project ](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project) github repository.

## Description

This project utilizes a kalman filter to estimate the state of a moving object of interest with:
 * noisy lidar measurements 
 * noisy radar measurements. 

This project involves the [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases).

## Building the code

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

##  Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

##  The implementation summary

The code follows standards of applying Kalman Filter which contain:
* Initialization step
* Prediction step
* Update step

The data that is received comes from two sources:
* Laser 
* Radar

Those two sources differ in terms of:
* Coordinate system (cartesian vs polar)
* Data (position vs position with speed)

That's why different transofmarions were applied for those two sources. The algorithm itself tires to combine measurment and prediction in terms of gaussian probability. More lecture about the theory can be viewed [here](https://en.wikipedia.org/wiki/Extended_Kalman_filter).


##  Metrics

The error was checked in terms of **RMSE** (Root Mean Squared Error) on Dataset1 and Dataset2 from "Term 2 Simulator" mentioned above.

The results on Dataset1:
* X: 0.0974
* Y: 0.0855
* VX: 0.4517
* VY: 0.4404

The results on Dataset2:
* X: 0.0726
* Y: 0.0965
* VX: 0.4216
* VY: 0.4932
