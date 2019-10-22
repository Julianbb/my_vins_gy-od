#include "feature_traker.h"





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



void FeatureTracker::setMask()
{
    if(FISHEYE)
    {
        fisheye_mask = cv::imread(FISHEYE_MASK, 0);
        mask = fisheye_mask.clone();
    }
    else
    {
        mask = cv::Mat(ROW, COL, CV_8UC1, 255);
    }

}


void FeatureTracker::rejectWithF()
{
    if(forw_pts.size() > 15)
    {
        TicToc t_f;
        vector<cv::Point2f> un_prev_pts(cur_pts.size()), un_forw_pts(cur_pts.size());

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
    }



    cur_pts = forw_pts;
    cur_img = forw_img;
}