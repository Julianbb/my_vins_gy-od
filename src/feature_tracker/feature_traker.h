#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include <eigen3/Eigen/Dense>
#include "tic_toc.h"
#include "../parameter.h"


using namespace std;
using namespace Eigen;
using namespace camodocal;



bool inBorder(const cv::Point2f& pts);
void reduceVector(vector<cv::Point2f>& pts, vector<uchar> status);
void reduceVector(vector<int>& v, vector<uchar> status);


class FeatureTracker
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FeatureTracker();

    void readImage(const cv::Mat &_img);

	void setMask();

	void addPoints();
    


    cv::Mat mask; 
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img;


    vector<cv::Point2f> n_pts;// 每一帧图像新提取的特征点
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    vector<int> ids;//能够被跟踪到的特征点的id
    vector<int> track_cnt;
    camodocal::CameraPtr m_camera;

    static int n_id; //用来做新特征点的id
};