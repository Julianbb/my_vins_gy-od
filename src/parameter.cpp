#include "parameter.h"


#define FILENAMEPATH_MAX 80
using namespace std;

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;


double rear_wheel_pulses;
double rear_track;
double right_rear_wheel_radius;
double left_rear_wheel_radius;
double odom_noise;

std::vector<Eigen::Matrix3d> ROC;
std::vector<Eigen::Vector3d>  TOC;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Matrix3d RIO;
Eigen::Vector3d TIO;
Eigen::Vector3d TOI;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
int LOOP_CLOSURE = 0;
int MIN_LOOP_NUM;
std::string CAM_NAMES_ESTIMATOR;   //add
std::string PATTERN_FILE;
std::string VOC_FILE;
std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
int IMAGE_ROW, IMAGE_COL;
std::string VINS_FOLDER_PATH;
int MAX_KEYFRAME_NUM;



//feature tracker section
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE_FEATURE_TRACKER;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
double FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;
float VISUALLOOKATX;
float VISUALLOOKATY;
float VISUALLOOKATZ;
Eigen::Matrix3d K;

bool exR_finished;

double triangulate_threshold;

void readParameters(const string & config_file)
{
    cv::FileStorage fsSettings(config_file.c_str(), cv::FileStorage::READ);
    if(fsSettings.isOpened())
    {
        std::cerr << "wrong path of config_file" << std::endl;
        cout<<VINS_FOLDER_PATH<<" "<<endl;
        return;
    }

    K = Eigen::Matrix3d::Identity();
    cv::FileNode n = fsSettings["projection_parameters"];
    double fx = static_cast<double>(n["fx"]);
    double fy = static_cast<double>(n["fy"]);
    double cx = static_cast<double>(n["cx"]);
    double cy = static_cast<double>(n["cy"]);

    K(0,0) = fx;
    K(0,2) = cx;
    K(1,1) = fy;
    K(1,2) = cy;

    fsSettings["triangulate_threshold"] >> triangulate_threshold;

    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    fsSettings["visualLookAtX"] >> VISUALLOOKATX;
    fsSettings["visualLookAtY"] >> VISUALLOOKATY;
    fsSettings["visualLookAtZ"] >> VISUALLOOKATZ;

    IMAGE_COL = fsSettings["image_width"];
    IMAGE_ROW = fsSettings["image_height"];

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];

    fsSettings["output_path"] >> VINS_RESULT_PATH;
    VINS_RESULT_PATH = VINS_FOLDER_PATH + VINS_RESULT_PATH;

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];


    ESTIMATE_EXTRINSIC = fsSettings["optimization_Toc_extrinsic"];

    {
        cv::Mat cv_T;
        fsSettings["I2OextrinsicTranslation"] >> cv_T;

        Eigen::Vector3d cur_eigen_T;
        cv::cv2eigen(cv_T, cur_eigen_T);
        TOI=cur_eigen_T;
    }


    for (int i = 0; i < NUM_OF_CAM; ++i)
    {
        string Rotation = "C" + to_string(i) + "2OextrinsicRotation";
        string Translation = "C" + to_string(i) + "2OextrinsicTranslation";

        cv::Mat cv_R, cv_T;
        fsSettings[Rotation] >> cv_R;
        fsSettings[Translation] >> cv_T;
        Eigen::Vector3d eigen_T;
        Eigen::Matrix3d eigen_R;

        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);

        ROC.push_back(eigen_R);//cam_to_im
        TOC.push_back(eigen_T);
    }



    int boolfinished = fsSettings["exR_finished"];
    exR_finished = boolfinished;

    
    if(exR_finished)
    {
        RIC.resize(NUM_OF_CAM);
        TIC.resize(NUM_OF_CAM);

        cv::Mat cv_R;
        fsSettings["C02IMUextrinsicRotation"] >> cv_R;
        Eigen::Matrix3d eigen_R;
        cv::cv2eigen(cv_R, eigen_R);

        RIC[0] = eigen_R;
        RIO = RIC[0] * ROC[0].transpose();

        TIO = -RIO*TOI;
        TIC[0] = RIO * TOC[0] + TIO;

        if(NUM_OF_CAM > 1)
        {
            for (int i = 0; i <NUM_OF_CAM ; ++i)
            {
                if(i == 0)
                    continue;
                RIC[i] = RIO * ROC[i];
                TIC[i] = RIO * TOC[i] + TIO;
            }
        }
    } 
    else
    {
        for (int i = 0; i < NUM_OF_CAM; ++i)
        {
            RIC.push_back(Eigen::Matrix3d::Identity());
            TIC.push_back(Eigen::Vector3d::Zero());

        }

        {
            RIO = Eigen::Matrix3d::Identity();
            TIO = Eigen::Vector3d::Zero();
        }
    }


    LOOP_CLOSURE = fsSettings["loop_closure"];
    if (LOOP_CLOSURE == 1)
    {
        fsSettings["voc_file"] >> VOC_FILE;;
        fsSettings["pattern_file"] >> PATTERN_FILE;
        VOC_FILE = VINS_FOLDER_PATH + VOC_FILE;
        PATTERN_FILE = VINS_FOLDER_PATH+ PATTERN_FILE;
        MIN_LOOP_NUM = fsSettings["min_loop_num"];
        CAM_NAMES_ESTIMATOR = config_file;   //add
    }



    INIT_DEPTH = 5.0;
    MAX_KEYFRAME_NUM = 4000;

    // feature tracker
    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "/config/fisheye_mask.jpg";
    cout<<FISHEYE_MASK<<endl;
    CAM_NAMES.push_back(config_file);
    //ROS_INFO_STREAM("CAM_NAMES" << CAM_NAMES.front());
    WINDOW_SIZE_FEATURE_TRACKER = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 316.6;
    PUB_THIS_FRAME = false;

    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;
    if (FREQ == 0)
        FREQ = 100;


    ////wheel
    rear_wheel_pulses = 78;
    rear_track = 1.56;
    right_rear_wheel_radius = 0.315/0.960260;
    left_rear_wheel_radius = 0.315/0.958427;
    odom_noise= 0.001;




    fsSettings.release();
}