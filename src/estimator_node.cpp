#include "feature_tracker/feature_tracker.h"
#include "parameter.h"

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>


#include <mutex>
#include <thread>
#include <fstream>
#include <iostream>
#include <chrono>
using namespace std;


FeatureTracker trackerData[NUM_OF_CAM];

bool first_image_flag = true;
double first_image_time;
double last_imudata[7];
double last_dwsdata[5];

void tracker()
{

}

void LoadImus(ifstream & fImus, const ros::Time &imageTimestamp)
{
    while(!fImus.eof())
    {
        string s;
        getline(fImus, s);
        if(!s.empty())
        {
            char c = s[0];
            if(c < '0' || c > '9')
                continue;
            
            stringstream ss;
            ss << s;
            double tmpd;
            int cnt = 0;
            double data[7];

            while(ss >> tmpd)
            {
                data[cnt++] = tmpd;
                if (cnt == 7)
                    break;
                
                if(ss.peek() == ',' || ss.peek() == ' ')
                    ss.ignore();
            }
            sensor_msgs::ImuPtr imudata(new sensor_msgs::Imu);
            imudata->angular_velocity.x = data[1];
            imudata->angular_velocity.y = data[2];
            imudata->angular_velocity.z = data[3];
            imudata->linear_acceleration.x = data[4];
            imudata->linear_acceleration.y = data[5];
            imudata->linear_acceleration.z = data[6];

            uint32_t  sec = data[0];
            uint32_t nsec = (data[0]-sec)*1e9;
            imudata->header.stamp = ros::Time(sec,nsec);

            if(imudata->header.stamp > imageTimestamp && last_imudata[0] > 0.00001)
            {
                double dt_1 = imageTimestamp.toSec() - last_imudata[0];
                double dt_2 = imudata->header.stamp.toSec() - imageTimestamp.toSec();
                assert(dt_1 >= 0);
                assert(dt_2 >= 0);
                assert(dt_1 + dt_2 > 0);

                double w1 = dt_2 / (dt_1 + dt_2);
                double w2 = dt_1 / (dt_2 + dt_1);

                double dx = w1 * last_imudata[1] + w2 * imudata->linear_acceleration.x;
                double dy = w1 * last_imudata[2] + w2 * imudata->linear_acceleration.y;
                double dz = w1 * last_imudata[3] + w2 * imudata->linear_acceleration.z;
                double rx = w1 * last_imudata[4] + w2 * imudata->angular_velocity.x;
                double ry = w1 * last_imudata[5] + w2 * imudata->angular_velocity.y;
                double rz = w1 * last_imudata[6] + w2 * imudata->angular_velocity.z;

                sensor_msgs::ImuPtr syn_imudata(new sensor_msgs::Imu);

                syn_imudata->angular_velocity.x = rx;
                syn_imudata->angular_velocity.y = ry;
                syn_imudata->angular_velocity.z = rz;
                syn_imudata->linear_acceleration.x = dx;
                syn_imudata->linear_acceleration.y = dy;
                syn_imudata->linear_acceleration.z = dz;

                uint32_t  sec = imageTimestamp.toSec();
                uint32_t nsec = (imageTimestamp.toSec()-sec)*1e9;
                syn_imudata->header.stamp = ros::Time(sec,nsec);
                                    
                imu_callback(syn_imudata);
            }
            else
            {
                last_imudata[0] = imudata->header.stamp.toSec();
                last_imudata[1]= imudata->linear_acceleration.x;
                last_imudata[2]= imudata->linear_acceleration.y;
                last_imudata[3]= imudata->linear_acceleration.z;
                last_imudata[4]= imudata->angular_velocity.x;
                last_imudata[5]= imudata->angular_velocity.y;
                last_imudata[6]= imudata->angular_velocity.z;
            }
            imu_callback(imudata);
            if(imudata->header.stamp > imageTimestamp )
                break;

        }
    }


}



void LoadWheel(ifstream & fwheel, const ros::Time &imageTimestamp)
{
    while(!fwheel.eof())
    {
        string s;
        getline(fwheel, s);
        if(!s.empty())
        {
            char c = s[0];
            if(c < '0' || c > '9')
                continue;
            
            stringstream ss;
            ss << s;

            double tmpd;
            int cnt =0; 
            double data[3];
            while(ss >> tmpd)
            {
                data[cnt++] = tmpd;
                if(cnt == 3)
                    break;
                if(ss.peek() == ',' || ss.peek() == ' ')
                    ss.ignore();
            }

            ZM_slam::dws_infoPtr Wheeldata(new ZM_slam::dws_info);
            Wheeldata->left_front = data[1];
            Wheeldata->right_front = data[2];
            Wheeldata->left_rear = data[1];
            Wheeldata->right_rear = data[2];

            uint32_t  sec = data[0];
            uint32_t nsec = (data[0]-sec)*1e9;
            Wheeldata->header.stamp = ros::Time(sec,nsec);

            if(Wheeldata->header.stamp > imageTimestamp && last_dwsdata[0] >= 0.00001)
            {
                double dt_1 = imageTimestamp - last_dwsdata[0];
                double dt_2 = Wheeldata->header.stamp.toSec() - imageTimestamp;
            }
        }
    }

}


void feature_callback()
{
    
}


void process()
{


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    //registerPub(n); 

    string data_folder = argv[2];
    ifstream fimus;
    fimus.open(data_folder + "/imu_data.txt");
    ifstream fwheel;
    fwheel.open(data_folder + "/dws_data.txt");



    VINS_FOLDER_PATH = string(argv[3]);

    readParameters(argv[1]);
//    estimator.setParameter();
    // for (int i = 0; i < NUM_OF_CAM; i++)
    //     trackerData[i].readIntrinsicParameter(CAM_NAMES[0]); //设置相机参数



    std::thread measurement_process{process};
    std::thread loop_detection, pose_graph, depth_estimate;


    vector<cv::VideoCapture> cap;
    vector<string> aviname;
    for(int i=0; i<NUM_OF_CAM; i++)
    {
        string filename = folder + "/" +to_string(i) +".avi";
        aviname.push_back(filename);
        cap.push_back(cv::VideoCapture(filename));
    }


    ifstream f_img_time;
    f_img_time.open(data_folder + "/frame_times.txt");

    if (LOOP_CLOSURE)
    {
        loop_detection = std::thread(process_loop_detection);
        pose_graph = std::thread(process_pose_graph);
        loop_detection.detach();
        pose_graph.detach();

	    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(CAM_NAMES_ESTIMATOR);
    }

    int imageNum = cap[0].get(CV_CAP_PROP_FRAME_COUNT); //得到视频文件的帧数


    if(!generate_undistort_map())
    {
        return EXIT_FAILURE;
    }

    for(int ni=0; ni<imageNum-1; ni++)//根据cap[0]取的帧数， cap[0]时间最短
    {

        double tframe;
        if(!f_img_time.eof())
        {
            string s;
            getline(f_img_time, s);
            stringstream ss;
            ss << s;
            ss >> tframe;
        }

        uint32_t sec = tframe;
        uint32_t nsec = (tframe-sec)*1e9;
        ros::Time image_timestamp = ros::Time(sec, nsec);


        if(first_image_flag) 
        {
            first_image_flag = false;
            first_image_time = image_timestamp.toSec();
            continue;
        }

        if (round(1.0 * pub_count / (image_timestamp.toSec() - first_image_time)) <= FREQ)
        {
            PUB_THIS_FRAME = true;
            if (abs(1.0 * pub_count / (image_timestamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
            {
                first_image_time = image_timestamp.toSec();
                pub_count = 0;
            }
        }
        else
            PUB_THIS_FRAME = false;


        if(PUB_THIS_FRAME)
        {
             
            LoadImus(fImus, image_timestamp);
            LoadWheel(fwheel,image_timestamp);
            
        }

        vector<cv::Mat> total_img(NUM_OF_CAM);
        vector<sensor_msgs::PointCloudConstPtr> featuremsg(NUM_OF_CAM);
        std::thread tracker_thread[NUM_OF_CAM];

        for(int i = 0; i < NUM_OF_CAM; i++)
        {
            tracker_thread[i] = std::thread(tracker, std::ref(total_img[i]), cap[i], 
                                            *corrector, std::ref(featuremsg[i]), std::ref(trackerData[i]),
                                            image_timestamp);
        }
        for(int i = 0; i < NUM_OF_CAM; i++)
        {
            tracker_thread[i].join();
        }


        if(PUB_THIS_FRAME)
        {
            cout<<"tracking: "<<image_timestamp.toSec()<<endl;
            pub_count++;

            feature_callback(featuremsg);  //保存feature

            if(LOOP_CLOSURE)
            {
                i_buf.lock();
                for (int i = 0; i <NUM_OF_CAM ; ++i)
                {
                    if(total_img[i].empty()){
                        cerr << endl << "Failed to load image: " << image_timestamp.toSec() <<endl;
                        return EXIT_FAILURE;
                    }
                    image_buf[i].push(make_pair(total_img[i], image_timestamp.toSec())); //保存图像
                }
                i_buf.unlock();

            }
        }
    }

    measurement_process.join();

    running_flag = false;
    return EXIT_SUCCESS;
}