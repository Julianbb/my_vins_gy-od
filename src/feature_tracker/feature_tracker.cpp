#include "feature_tracker.h"


int FeatureTracker::n_id = 0;


bool inBorder(const cv::Point2f& pts)
{
    const int BORDER_SIZE = 1; //预留一个像素出来
    int x = cvRound(pts.x);
    int y = cvRound(pts.y);

    return (x>=BORDER_SIZE && x<=COL-BORDER_SIZE)&&(y>=BORDER_SIZE && y<=COL-BORDER_SIZE);
}



void reduceVector(vector<cv::Point2f>& pts, vector<uchar> status)
{

    size_t N = status.size();

    int j = 0;
    for(size_t i=0; i<N; i++)
    {
        if(status[i])
        {
            pts[j] = pts[i];
            j++;
        }  
    }
    pts.resize(j);
}


void reduceVector(vector<int>& v, vector<uchar> status)
{
    size_t N = status.size();

    int j = 0;
    for(size_t i=0; i<N; i++)
    {
        if(status[i])
        {
            v[j++] = v[i];
        }
    }
    v.resize(j);
}




FeatureTracker::FeatureTracker()
{
   
}


//每次跟踪图像结束的时候都需要重新setMask, 因为提取强角点需要将已经跟踪到的点附近MIN_DIST设置为mask
//以至于再检查强角点的时候，放弃这些区域
void FeatureTracker::setMask()
{
    //每次重新置位mask
    if(FISHEYE)
    {
        fisheye_mask = cv::imread(FISHEYE_MASK, 0);
        mask = fisheye_mask.clone();
    }
    else
    {
        mask = cv::Mat(ROW, COL, CV_8UC1, 255);
    }


    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
    for(int i=0; i<forw_pts.size(); i++)
    {
        cnt_pts_id.push_back(make_pair(track_cnt[i],make_pair(forw_pts[i], ids[i])));
    }

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), []
    (const pair<int, pair<cv::Point2f, int>>& a, const pair<int, pair<cv::Point2f, int>>& b)
    {
        return a.first > b.first;
    });

    forw_pts.clear();
    track_cnt.clear();
    ids.clear();

    for(auto& tmp:cnt_pts_id)
    {
        if(mask.at<uchar>(tmp.second.first) == 255)
        {
            forw_pts.push_back(tmp.second.first);
            track_cnt.push_back(tmp.first);
            ids.push_back(tmp.second.second);
            cv::circle(mask, tmp.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::ExtractEdgeFeature(std::vector<cv::Point2f>& total_pts, int NeedNum)
{
    cv::Mat cur_Mask = mask.clone();
    cv::Mat img = forw_img.clone();
    cv::Mat gradx = cv::Mat::zeros( forw_img.rows, forw_img.cols, CV_32F);
    cv::Mat grady = cv::Mat::zeros(forw_img.rows, forw_img.cols, CV_32F);
    cv::Mat mag =  cv::Mat::zeros(forw_img.rows, forw_img.cols, CV_32F);


    cv::GaussianBlur( forw_img, img, cv::Size( 3, 3 ), 0, 0 );//高斯滤波器（GaussianFilter）对图像进行平滑处理。
    cv::Scharr(img, gradx, CV_32F, 1, 0, 1/32.0);//边缘检测
    cv::Scharr(img, grady, CV_32F, 0, 1, 1/32.0);
    cv::magnitude(gradx,grady,mag);//突出了边缘

    cv::Mat canny;
    cv::Canny(img , canny, 30, 50);


    //以前取过的区域就忽略
    if(total_pts.size() != 0)
    {
        for(int k = 0; k < (int)total_pts.size(); ++k)
        {
            cv::circle(cur_Mask, cv::Point(total_pts[k].x, total_pts[k].y), MIN_DIST, 0, -1);
        }
    }

    //在每个MIN_DIST的区域内找最大的
    for(size_t i=1; i+MIN_DIST<forw_img.rows-1; i=i+MIN_DIST)
    {
        for(size_t j=1; j+MIN_DIST<forw_img.cols-1; j=j+MIN_DIST)
        {
            float max_grad = 0;
            int maxgrad_x = 0;
            int maxgrad_y = 0;

            for(int y=0; y<MIN_DIST; ++y)
            {
                for(int x=0; x<MIN_DIST; ++x)
                {
                    if(cur_Mask.ptr<float>(i+y)[j+x] != 255) continue;

                    float tmp = mag.ptr<float>(i+y)[j+x];
                    if(canny.ptr<uchar>(i+y)[j+x] == 0) continue;

                    if(tmp > max_grad)
                    {
                        maxgrad_x = j+x;
                        maxgrad_y = i+y;
                        max_grad = tmp;
                    }
                }
            }
            float edge_threshold =4.0;
            {
                if(max_grad > edge_threshold)
                {
                    total_pts.push_back(cv::Point2f((float)maxgrad_x, (float)maxgrad_y));
                    cv::circle(cur_Mask, cv::Point2f((float)maxgrad_x, (float)maxgrad_y), MIN_DIST, 0, -1);
                    if(total_pts.size() >= NeedNum)
                        return;
                }
            }

        }
    }


}

void FeatureTracker::rejectWithF()
{
    if(forw_pts.size() > 15)
    {
        TicToc t_f;
        vector<cv::Point2f> un_prev_pts(cur_pts.size()), un_forw_pts(cur_pts.size());

        // 去畸变
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y),tmp_p); // 去畸变到相机系
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x()/tmp_p.z() + COL/2;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y()/tmp_p.z() + ROW/2;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        double focal = 1.0;
        cv::Point2f pp(0,0);
        cv::Mat E = cv::findEssentialMat(un_prev_pts, un_forw_pts, focal, pp, cv::RANSAC, 0.99, F_THRESHOLD, status );
    
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
    }


}

void FeatureTracker::addPoints()
{
    for(auto& p: n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);

    }
}

//跟新第i个特征点的id
bool FeatureTracker::updateID(unsigned int i)
{
    if(i< ids.size())
    {
        if(ids[i] == -1)
            ids[i] = n_id++;
        
        return true;
    }
    else
        return false;
}




void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    cout << "reading paramerter of camera " <<  calib_file.c_str() <<endl;
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}


//显示去畸变的图
void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat un_img(ROW+600, COL+600, CV_8UC1, cv::Scalar(0)); //往中间显示
    vector<Eigen::Vector2d> distorted_p, undistorted_p;
    for(int i=0; i<ROW; i++)
    {
        for(int j=0; j<COL; j++)
        {
            Eigen::Vector2d p(j,i);
            Eigen::Vector3d P;
            
            m_camera->liftProjective(p, P);

            distorted_p.push_back(p);
            undistorted_p.push_back(Eigen::Vector2d(P.x()/P.z(), P.y()/P.z()));
        }
    }


    for(int i=0; i<(int)undistorted_p.size(); ++i)
    {
        //转到像素
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistorted_p[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistorted_p[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;

        //判断在不在显示范围内
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            un_img.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distorted_p[i].y(), distorted_p[i].x());
        }
        else
        {
        }
    }
    cv::imshow(name, un_img);
    cv::waitKey(0);

}

//返回是归一化平面的点(已矫正)
vector<cv::Point2f> FeatureTracker::undistortedPoints()
{
    vector<cv::Point2f> un_points;
    un_points.reserve(cur_pts.size());
    for(auto& pts: cur_pts)
    {
        Eigen::Vector2d p(pts.x, pts.y);
        Eigen::Vector3d P;
        m_camera->liftProjective(p, P);
        un_points.push_back(cv::Point2f(P.x()/P.z(), P.y()/P.z()));
    }

    return un_points;

}




void FeatureTracker::readImage(const cv::Mat &_img)
{
    cv::Mat img;
    TicToc t_r;
    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
    }
    else
        img = _img;


    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

    if(cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;

        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(31, 31), 3);

//测试使用
        // cv::Mat imshow(cur_img.rows*2, cur_img.cols, CV_8UC3);
        // cv::Mat cur_show,forw_show;
        // cv::cvtColor(cur_img, cur_show, cv::COLOR_GRAY2BGR);
        // cv::cvtColor(forw_img, forw_show, cv::COLOR_GRAY2BGR);
        // cur_show.copyTo(imshow ( cv::Rect (0, 0, cur_show.cols, cur_show.rows ) )); // TODO: copy可以这么写??
        // forw_show.copyTo(imshow ( cv::Rect (0, cur_show.rows, forw_show.cols, forw_show.rows ) ) );


        for(int i=0; i<forw_pts.size(); i++)
        {
            if(status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;

            reduceVector(prev_pts, status);
            reduceVector(cur_pts, status);
            reduceVector(forw_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
        }

    }

    if(PUB_THIS_FRAME)
    {
        rejectWithF();
        for(auto& n:track_cnt)
            n++;

        TicToc t_m;
        setMask();
        TicToc t_t;

        int n_max_cnt =  MAX_CNT - static_cast<int>(forw_pts.size());
        if(n_max_cnt>0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;

            cv::goodFeaturesToTrack(forw_img, n_pts, n_max_cnt/2, 0.01, MIN_DIST, mask);
            int edgepoint = n_max_cnt -n_pts.size(); // 剩下用边提取点
            ExtractEdgeFeature(n_pts,edgepoint);
        }
        else
            n_pts.clear();

        TicToc t_a;
        addPoints(); //TODO: n_pts每次用完不用clear???
        prev_img = forw_img;
        prev_pts = forw_pts;
    }



    cur_pts = forw_pts;
    cur_img = forw_img;
}