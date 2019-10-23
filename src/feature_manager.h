#include <iostream>
#include <vector>
#include <list>
#include <map>
using namespace std;

#include "parameter.h"
#include <eigen3/Eigen/Dense>
using namespace Eigen;
#include <assert.h>





class FeaturePerFrame
{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FeaturePerFrame(Eigen::Vector3d _point)
    {
        double z = _point(2);
        point = _point/z;
    }

    Eigen::Vector3d point;
    double z;
    bool is_used;
    double parallax;
    MatrixXd A;
    VectorXd b;
    double dep_gradient;
};






class FeaturePerId
{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    const int feature_id;
    int start_frame; //不是全局，是滑窗内的索引
    vector<FeaturePerFrame> feature_per_frame; //观测到该feature的共视帧集合


    int used_num;
    bool is_outlier;
    bool is_margin;
    double estimated_depth; //逆深度
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

    Vector3d gt_p;

    FeaturePerId(int _feature_id, int _start_frame):feature_id(_feature_id), start_frame(_start_frame),
    used_num(0), estimated_depth(0), solve_flag(0){}

    int endFrame(); 
};







class FeatureManager
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FeatureManager(Matrix3d _Rs[]);

    void setRic(Matrix3d _ric);
    void clearState(); //清除feature
    int getFeatureCount();
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Vector3d>>> &image);
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);
    void setDepth(const VectorXd &x);
    void removeFailures();
    void clearDepth(const VectorXd &x);
    VectorXd getDepthVector();
    void triangulate(Vector3d Ps[], Vector3d toc, Matrix3d roc);  
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);  
    void removeOutlier();
    void removeFront(int frame_count);
    void removeBack();
    list<FeaturePerId> feature;


    int last_track_num;
    bool isold;

private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    const Matrix3d *Rs;
    Matrix3d ric;
};