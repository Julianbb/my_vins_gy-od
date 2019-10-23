#include "feature_manager.h"


int FeaturePerId::endFrame()
{
    return start_frame+feature_per_frame.size()-1;
}





FeatureManager::FeatureManager(Matrix3d _Rs[]):Rs(_Rs)
{
    ric.setIdentity();  
}

void FeatureManager::setRic(Matrix3d _ric)
{
    ric = _ric;
}

void FeatureManager::clearState()
{
    feature.clear();
}


//返回合格的feature的个数
int FeatureManager::getFeatureCount()
{
    int cnt = 0;

    for(auto& it:feature)
    {
        int used_num = it.feature_per_frame.size();

        if(used_num >= 2 && it.start_frame < WINDOW_SIZE-2)
        {
            cnt++;
        }
    }
    
    return cnt;
}


//取该特征点在两帧(共视倒数第二和倒数第三)的归一化平面上的坐标点的距离,
double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    const FeaturePerFrame& frame1 = it_per_id.feature_per_frame[frame_count-2-it_per_id.start_frame];
    const FeaturePerFrame& frame2 = it_per_id.feature_per_frame[frame_count-1-it_per_id.start_frame];


    double ans = 0; 
    Vector3d p1 = frame1.point;
    Vector3d p2 = frame2.point;
    Vector3d p_1_comp = p1; //某种补偿策略，这里没有用

    double u_1 = p1(0);
    double v_1 = p1(1);

    double u_2 = p2(0)/p2(2);
    double v_2 = p2(1)/p2(2);

    double u_1_comp = p_1_comp(0)/p_1_comp(2);
    double v_1_comp = p_1_comp(1)/p_1_comp(2);

    double du = u_1-u_2;
    double dv = v_1-v_2;

    double du_comp = u_1_comp - u_2;
    double dv_comp = v_1_comp - v_2;


    ans = max(ans, sqrt(min(du*du+dv*dv, du_comp*du_comp+dv_comp*dv_comp)));
}


// image: frame_count这一帧里面的feature,  //TODO: 结构<feature_id, < ??,归一化点>>
bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Vector3d>>> &image)
{

    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;//这次跟踪到的点的次数
    
    for(auto& id_pts:image) //先将　frame_count中的特征点　image跟新到 feature中
    {
        FeaturePerFrame f_per_f = id_pts.second[0].second; //TODO: 这里为什么只去 second[0]?
        int feature_id = id_pts.first;

        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId& it)
        {
            return it.feature_id == feature_id;
        });


        if(it == feature.end())//新点
        {
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_f);
        }
        else if(it->feature_id == feature_id)//以前就有，跟踪上了
        {
            it->feature_per_frame.push_back(f_per_f);
            last_track_num++;
        }
    }

    //如果窗内的帧数 < 2 ,或者这次跟踪到的点　< 20
    if(frame_count < 2 || last_track_num < 20)
    {
        isold = true;
        return true;
    }
        
    // 遍历feature, 计算frame_count往前　倒数一帧和倒数第二帧　的平均视差
    for(auto& it_per_id:feature)
    {
        if(it_per_id.start_frame <= frame_count-2 &&  //起始帧　< 倒数第二帧
           it_per_id.start_frame + (int)it_per_id.feature_per_frame.size()-1 >= frame_count-1)//终止帧 >＝　倒数第二帧
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        isold = true;
        return true;
    }
    else
    {
        isold = parallax_sum / parallax_num >= MIN_PARALLAX;
        return parallax_sum / parallax_num >= MIN_PARALLAX; //平均视差
    }
}



//得到这两帧的匹配
vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto& it : feature)
    {
        if(frame_count_l >= it.start_frame && frame_count_r <= it.endFrame())
        {
            Vector3d a = it.feature_per_frame[frame_count_l-it.start_frame].point;
            Vector3d b = it.feature_per_frame[frame_count_r-it.start_frame].point;

            corres.push_back(make_pair(a,b));
        }
    }

    return corres;
}

// 设置特征点的逆深度值
void FeatureManager::setDepth(const VectorXd &x)
{
    int index_dep = -1;
    for(auto& it_per_id: feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if(!(it_per_id.used_num>=2 && it_per_id.start_frame < WINDOW_SIZE-2))
            continue;
        
        it_per_id.estimated_depth = 1.0/x(++index_dep);

        if(it_per_id.estimated_depth > 0)
        {
            it_per_id.solve_flag = 1;
        }
        else
        {
            it_per_id.solve_flag = 2;
        }
        
    }
}


void FeatureManager::removeFailures()
{
    for(auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it=it_next)
    {
        it_next++;  
        if(it->solve_flag == 2)
            feature.erase(it); //注意这里， erase返回是下一个 iterator
    }
}



void FeatureManager::clearDepth(const VectorXd &x)
{
    int index_dep = -1;
    for(auto& it_per_id: feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if(it_per_id.used_num < 2 || it_per_id.start_frame >= WINDOW_SIZE-2)
            continue;
        
        it_per_id.estimated_depth = 1.0/x(++index_dep);
    }
}


//返回特征点的深度值(不是逆深度，向量形式)
VectorXd FeatureManager::getDepthVector()
{
    int index_dep = -1;
    VectorXd tmp_depth(getFeatureCount());
    for(auto& it_per_id: feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        tmp_depth(++index_dep) = 1.0 / it_per_id.estimated_depth;
    }
           
    
}


void FeatureManager::triangulate(Vector3d Ps[], Vector3d toc, Matrix3d roc)
{



}



void FeatureManager::removeOutlier()
{
    for(auto it = feature.begin(), it_next=feature.begin(); it!=feature.end(); it = it_next)
    {
        it_next++;
        if(it->used_num != 0 && it->is_outlier == true)
        {
            feature.erase(it);
        }
    }
}

void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{



}


void FeatureManager::removeFront(int frame_count)
{

}


//滑窗的时候，删除旧帧feature处理
void FeatureManager::removeBack()
{
    for(auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if(it->start_frame > 0)
        {
            it->start_frame--;
        }
        else
        {   
            if(it->feature_per_frame.size() == 0)
            {
                feature.erase(it);
                continue;
            }               
            //把每个feature_per_id最前面的帧删除
            it->feature_per_frame.erase(it->feature_per_frame.begin()); 

        }
    }
}