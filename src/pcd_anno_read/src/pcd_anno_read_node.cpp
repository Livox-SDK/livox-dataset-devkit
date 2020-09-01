#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"

#include "pcl/conversions.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PCLPointCloud2.h"
#include <pcl/common/transforms.h>
#include <time.h>
#include <memory.h>
#include <math.h>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

using namespace std;
ros::Publisher pcd_pub;
ros::Publisher anno_pub;
ros::Publisher txt_pub;

ros::Timer timer;

#define PI 3.1415926

string root_path = "/home/dji/桌面/data_set/livox_dataset_v1.0";
int demo_frequence = 2; //Hz
int preBoxNum = 0;
float parameter[6][6] = {{0, 0.43, 0, 0, 0, 1.909},
                         {-0.27, 8.56, 69.98, -0.001, 0.338, 1.909},
                         {-0.67, 9.43, 138.84, -1.109, 0.15, 1.909},
                         {0.5, 9.06, 218.69, -1.2, -0.149, 1.909},
                         {-0.2, 8.33, -70.01, -0.009, -0.301, 1.909},
                         {0.91, 0.67, 0.06, -0.037, 0.019, 1.909}};

void pcd_rotation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int n)
{
    Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
    transform_1.translation() << parameter[n - 1][3], parameter[n - 1][4], parameter[n - 1][5];
    transform_1.rotate(Eigen::AngleAxisf(M_PI * parameter[n - 1][2] / 180, Eigen::Vector3f::UnitZ()));
    transform_1.rotate(Eigen::AngleAxisf(M_PI * parameter[n - 1][1] / 180, Eigen::Vector3f::UnitY()));
    transform_1.rotate(Eigen::AngleAxisf(M_PI * parameter[n - 1][0] / 180, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*cloud, *cloud, transform_1);
}

void pcd_read(string pcd_name, string data_seq)
{
    pcl::PointCloud<pcl::PointXYZI> cloud_all;
    sensor_msgs::PointCloud2 msg;
    for (int p = 1; p < 7; p++)
    {
        string pcd_path = root_path + "/data/" + data_seq + "/lidar/" + to_string(p) + "/" + pcd_name;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud) == -1)
        {
            PCL_ERROR("Couldn't read pcd file\n");
            return;
        }
        pcd_rotation(cloud, p);
        cloud_all += *cloud;
    }
    pcl::PointCloud<pcl::PointXYZRGB> cloud2;
    pcl::RGB rgb;
    cloud2.width = cloud_all.points.size();
    cloud2.height = 1;
    cloud2.points.resize(cloud2.width * cloud2.height);
    for (size_t i = 0; i < cloud_all.points.size(); ++i)
    {
        cloud2.points[i].x = cloud_all.points[i].x;
        cloud2.points[i].y = cloud_all.points[i].y;
        cloud2.points[i].z = cloud_all.points[i].z;
        float intensity = cloud_all.points[i].intensity;
        if (intensity < 30.0)
        {
            int green = intensity * 255 / 30;
            rgb.r = 0;
            rgb.g = green & 0xff;
            rgb.b = 0xff;
        }
        else if (intensity < 90.0)
        {
            int blue = (90 - intensity) * 255 / 60;
            rgb.r = 0;
            rgb.g = 0xff;
            rgb.b = blue & 0xff;
        }
        else if (intensity < 150.0)
        {
            int red = (intensity - 90) * 255 / 60;
            rgb.r = red & 0xff;
            rgb.g = 0xff;
            rgb.b = 0;
        }
        else
        {
            int green = (255 - intensity) * 255 / (256 - 150);
            rgb.r = 0xff;
            rgb.g = green & 0xff;
            rgb.b = 0;
        }
        cloud2.points[i].r = rgb.r;
        cloud2.points[i].b = rgb.b;
        cloud2.points[i].g = rgb.g;
    }
    pcl::toROSMsg(cloud2, msg);
    msg.header.frame_id = "origin";
    pcd_pub.publish(msg);
}
void anno_read(string anno_name, string data_seq, int cur_num, int total_num)
{
    string gt_path = root_path + "/label/" + data_seq + "/lidar/all/" + anno_name;
    vector<string> gt_cls, state, visualable;
    vector<float> gt_x, gt_y, gt_z, gt_l, gt_w, gt_h, gt_yaw;
    vector<int> tracking_id;
    ifstream in(gt_path.c_str());
    string line;
    int obj_num = 0;
    while (getline(in, line))
    {
        stringstream word(line);
        string x;
        word >> x;
        tracking_id.push_back(stoi(x));
        word >> x;
        gt_cls.push_back(x);
        word >> x;
        visualable.push_back(x);
        word >> x;
        state.push_back(x);
        word >> x;
        gt_x.push_back(atof(x.c_str()));
        word >> x;
        gt_y.push_back(atof(x.c_str()));
        word >> x;
        gt_z.push_back(atof(x.c_str()));
        word >> x;
        gt_l.push_back(atof(x.c_str()));
        word >> x;
        gt_w.push_back(atof(x.c_str()));
        word >> x;
        gt_h.push_back(atof(x.c_str()));
        word >> x;
        gt_yaw.push_back(atof(x.c_str()));
        obj_num++;
    }
    in.close();
    ROS_INFO("current_frame:%s %d/%d   obj_num:%d", data_seq.c_str(), cur_num + 1, total_num, obj_num);
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::MarkerArray txt_array;
    visualization_msgs::Marker bbox_marker;
    visualization_msgs::Marker marker_txt;
    bbox_marker.header.frame_id = "origin";
    bbox_marker.header.stamp = ros::Time::now();
    bbox_marker.ns = "";
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 1.0f;
    bbox_marker.color.b = 1.0f;
    bbox_marker.color.a = 0.8f;
    bbox_marker.frame_locked = true;
    bbox_marker.lifetime = ros::Duration(5);
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;

    marker_txt.header.stamp = ros::Time();
    marker_txt.header.frame_id = "origin";
    marker_txt.ns = "basic_shapes";

    marker_txt.action = visualization_msgs::Marker::ADD;
    marker_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_txt.scale.z = 2.0;
    marker_txt.color.b = 1.0f;
    marker_txt.color.g = 1.0f;
    marker_txt.color.r = 1.0f;
    marker_txt.color.a = 1.0f;
    marker_txt.lifetime = ros::Duration(5);
    geometry_msgs::Pose pose;

    for (size_t i = 0; i < gt_cls.size(); i++)
    {
        bbox_marker.id = i;
        marker_txt.id = i;
        bbox_marker.pose.position.x = gt_x[i];
        bbox_marker.pose.position.y = gt_y[i];
        bbox_marker.pose.position.z = gt_z[i];
        bbox_marker.pose.orientation.x = 0.0;
        bbox_marker.pose.orientation.y = 0.0;
        bbox_marker.pose.orientation.z = sin(gt_yaw[i] / 2);
        bbox_marker.pose.orientation.w = cos(gt_yaw[i] / 2);
        bbox_marker.scale.x = gt_l[i];
        bbox_marker.scale.y = gt_w[i];
        bbox_marker.scale.z = gt_h[i];
        marker_array.markers.push_back(bbox_marker);
        pose.position.x = gt_x[i];
        pose.position.y = gt_y[i];
        pose.position.z = gt_z[i] + 1;
        marker_txt.text = ("<" + to_string(tracking_id[i]) + ">" + gt_cls[i] + to_string(sqrt(gt_x[i] * gt_x[i] + gt_y[i] * gt_y[i] + gt_z[i] * gt_z[i])).substr(0, 5) + "m" + "\n").c_str();
        marker_txt.pose = pose;
        txt_array.markers.push_back(marker_txt);
    }
    if (gt_cls.size() < preBoxNum)
    {
        for (int i = gt_cls.size(); i < preBoxNum; i++)
        {
            bbox_marker.id = i;
            bbox_marker.pose.position.x = 0;
            bbox_marker.pose.position.y = 0;
            bbox_marker.pose.position.z = 0;
            bbox_marker.pose.orientation.x = 0.0;
            bbox_marker.pose.orientation.y = 0.0;
            bbox_marker.pose.orientation.z = 0.0;
            bbox_marker.pose.orientation.w = 0.0;
            bbox_marker.scale.x = 1;
            bbox_marker.scale.y = 1;
            bbox_marker.scale.z = 1;
            bbox_marker.color.a = 0.0f;
            marker_array.markers.push_back(bbox_marker);

            marker_txt.id = i;
            marker_txt.color.r = 1;
            marker_txt.color.g = 1;
            marker_txt.color.b = 1;

            marker_txt.color.a = 0;
            marker_txt.scale.z = 1;

            marker_txt.pose.orientation.w = 1.0;
            marker_txt.pose.position.x = 0;
            marker_txt.pose.position.y = 0;
            marker_txt.pose.position.z = 0;

            marker_txt.text = "aa";
            txt_array.markers.push_back(marker_txt);
        }
    }
    marker_txt.id = gt_cls.size();
    marker_txt.scale.z = 3.0;
    marker_txt.color.b = 0.0f;
    marker_txt.color.g = 0.0f;
    marker_txt.color.r = 1.0f;
    marker_txt.color.a = 1.0;
    marker_txt.lifetime = ros::Duration(float(1.0 / demo_frequence));
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 1;
    marker_txt.text = ("current_frame:" + to_string(cur_num + 1) + "/" + to_string(total_num) + "\n").c_str();
    marker_txt.pose = pose;
    txt_array.markers.push_back(marker_txt);
    anno_pub.publish(marker_array);
    txt_pub.publish(txt_array);
    preBoxNum = gt_cls.size();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "read_pub");
    ros::NodeHandle n;
    pcd_pub = n.advertise<sensor_msgs::PointCloud2>("pcd_pointcloud", 10);
    anno_pub = n.advertise<visualization_msgs::MarkerArray>("boxes", 10);
    txt_pub = n.advertise<visualization_msgs::MarkerArray>("txtes", 10);
    vector<string> seqlist;
    ifstream inf;
    inf.open(root_path + "/namelist.txt");
    string s;
    while (getline(inf, s))
    {
        seqlist.push_back(s);
    }
    inf.close();
    ros::Rate loop_rate(demo_frequence);

    vector<string> pcd_list;
    for (int i = 0; i < seqlist.size(); i++)
    {
        string path = root_path + "/data/" + seqlist[i] + "/lidar/" + "pcd_list.txt";
        cout << "Loading data from >>>> " << seqlist[i] << endl;
        ifstream in(path);
        string line;
        if (in)
        {
            while (getline(in, line))
            {
                pcd_list.push_back(line);
            }
        }
        else
        {
            cout << "no such file" << endl;
        }
        cout << "total frame: " << pcd_list.size() << endl;
        for (int n = 0; n < pcd_list.size(); n++)
        {
            pcd_read(pcd_list[n], seqlist[i]);
            string anno_name = pcd_list[n].replace(13, 3, "txt");
            anno_read(anno_name, seqlist[i], n, pcd_list.size());
            ros::spinOnce();
            loop_rate.sleep();
            if (!ros::ok())
                return 0;
        }
        pcd_list.clear();
    }
    return 0;
}