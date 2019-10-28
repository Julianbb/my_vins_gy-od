#pragma once
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

const int WINDOW_SIZE = 10;
const int NUM_OF_CAM = 2;
const int NUM_OF_F = 3000;

//#define DEPTH_PRIOR
//#define GT
//#define UNIT_SPHERE_ERROR;
#define SAVE_POSE;

extern float VISUALLOOKATX;
extern float VISUALLOOKATY;
extern float VISUALLOOKATZ;
extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern Eigen::Vector3d G;



extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string VINS_RESULT_PATH;
extern std::string VINS_FOLDER_PATH;
extern int IMAGE_ROW, IMAGE_COL;

extern int LOOP_CLOSURE;
extern int MIN_LOOP_NUM;
extern int MAX_KEYFRAME_NUM;
extern std::string PATTERN_FILE;
extern std::string VOC_FILE;
extern std::string CAM_NAMES_ESTIMATOR;
extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;

//feature tracker section
extern int ROW;
extern int COL;
extern double FOCAL_LENGTH;


extern std::string IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;
extern int MAX_CNT;
extern int MIN_DIST;
extern  int WINDOW_SIZE_FEATURE_TRACKER;
extern int FREQ;
extern double F_THRESHOLD;
extern int SHOW_TRACK;
extern int STEREO_TRACK;
extern int EQUALIZE;
extern int FISHEYE;
extern bool PUB_THIS_FRAME;


void readParameters(const string & config_file);



enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_T = 3,
    SIZE_Q = 4,
//    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1,
    SIZE_BIAS = 3,
    SIZE_SLIP = 2

};



enum wheelStateOrder
{
    wheelO_P = 0,
    wheelO_R = 3,
    wheelO_Sr = 6,
    wheelO_Sl = 7
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_BG = 6
};
//enum StateOrder
//{
//    O_P = 0,
//    O_R = 3,
//    O_V = 6,
//    O_BA = 9,
//    O_BG = 12
//};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
