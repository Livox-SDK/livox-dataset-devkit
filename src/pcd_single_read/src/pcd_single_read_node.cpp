#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
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

using namespace cv;
using namespace std;
ros::Publisher pcd_pub;
ros::Publisher anno_pub;
ros::Publisher txt_pub;
image_transport::Publisher img_pub;
ros::Timer timer;

string root_path = "/home/dji/桌面/data_set/livox_dataset_v1.0";

float parameter[6][6] = {{0, 0.43, 0, 0, 0, 0},
                         {-0.27, 8.56, 0, 0, 0, 0},
                         {-0.67, 9.43, 0, 0, 0, 0},
                         {0.5, 9.06, 0, 0, 0, 0},
                         {-0.2, 8.33, 0, 0, 0, 0},
                         {0.91, 0.67, 0, 0, 0, 0}};
int preBoxNum = 0;
int demo_frequence = 2; //Hz
void pcd_read(string pcd_name, string data_seq, int lidar_num)
{
    sensor_msgs::PointCloud2 msg;
    string pcd_path = root_path + "/data/" + data_seq + "/lidar/" + to_string(lidar_num) + "/" + pcd_name;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read pcd file\n");
        return;
    }

    Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
    transform_1.translation() << parameter[lidar_num - 1][3], parameter[lidar_num - 1][4], parameter[lidar_num - 1][5];
    transform_1.rotate(Eigen::AngleAxisf(M_PI * parameter[lidar_num - 1][2] / 180, Eigen::Vector3f::UnitZ()));
    transform_1.rotate(Eigen::AngleAxisf(M_PI * parameter[lidar_num - 1][1] / 180, Eigen::Vector3f::UnitY()));
    transform_1.rotate(Eigen::AngleAxisf(M_PI * parameter[lidar_num - 1][0] / 180, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*cloud, *cloud, transform_1);

    pcl::PointCloud<pcl::PointXYZRGB> cloud2;
    pcl::RGB rgb;
    cloud2.width = cloud->points.size();
    cloud2.height = 1;
    cloud2.points.resize(cloud2.width * cloud2.height);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud2.points[i].x = cloud->points[i].x;
        cloud2.points[i].y = cloud->points[i].y;
        cloud2.points[i].z = cloud->points[i].z;
        float intensity = cloud->points[i].intensity;
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
geometry_msgs::Point transform2g(pcl::PointXYZ points)
{
    geometry_msgs::Point tmp_pos;
    tmp_pos.x = points.x;
    tmp_pos.y = points.y;
    tmp_pos.z = points.z;
    return tmp_pos;
}
void anno_read(string anno_name, string data_seq, int cur_num, int total_num, int lidar_num)
{
    string gt_path = root_path + "/label/" + data_seq + "/lidar/" + to_string(lidar_num) + "/" + anno_name;
    vector<string> gt_cls, state, visualable;
    float x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3,
        x4, y4, z4, x5, y5, z5, x6, y6, z6, x7, y7, z7;
    vector<float> gt_x, gt_y, gt_z, gt_l, gt_w, gt_h, gt_yaw;
    vector<string> tracking_id;
    ifstream in(gt_path.c_str());
    string line;
    int obj_num = 0;
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << parameter[lidar_num - 1][3], parameter[lidar_num - 1][4], parameter[lidar_num - 1][5];
    transform_2.rotate(Eigen::AngleAxisf(M_PI * parameter[lidar_num - 1][2] / 180, Eigen::Vector3f::UnitZ()));
    transform_2.rotate(Eigen::AngleAxisf(M_PI * parameter[lidar_num - 1][1] / 180, Eigen::Vector3f::UnitY()));
    transform_2.rotate(Eigen::AngleAxisf(M_PI * parameter[lidar_num - 1][0] / 180, Eigen::Vector3f::UnitX()));
    pcl::PointCloud<pcl::PointXYZ>::Ptr coordinate(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ coor;
    while (getline(in, line))
    {
        coordinate->points.clear();
        stringstream word(line);
        string x = "";
        word >> x;
        tracking_id.push_back(x);
        word >> x;
        gt_cls.push_back(x);
        word >> x;
        visualable.push_back(x);
        word >> x;
        state.push_back(x);
        word >> x;
        coor.x = (atof(x.c_str()));
        word >> x;
        coor.y = (atof(x.c_str()));
        word >> x;
        coor.z = (atof(x.c_str()));
        coordinate->points.push_back(coor);

        word >> x;
        coor.x = (atof(x.c_str()));
        word >> x;
        coor.y = (atof(x.c_str()));
        word >> x;
        coor.z = (atof(x.c_str()));
        coordinate->points.push_back(coor);

        word >> x;
        coor.x = (atof(x.c_str()));
        word >> x;
        coor.y = (atof(x.c_str()));
        word >> x;
        coor.z = (atof(x.c_str()));
        coordinate->points.push_back(coor);

        word >> x;
        coor.x = (atof(x.c_str()));
        word >> x;
        coor.y = (atof(x.c_str()));
        word >> x;
        coor.z = (atof(x.c_str()));
        coordinate->points.push_back(coor);

        word >> x;
        coor.x = (atof(x.c_str()));
        word >> x;
        coor.y = (atof(x.c_str()));
        word >> x;
        coor.z = (atof(x.c_str()));
        coordinate->points.push_back(coor);

        word >> x;
        coor.x = (atof(x.c_str()));
        word >> x;
        coor.y = (atof(x.c_str()));
        word >> x;
        coor.z = (atof(x.c_str()));
        coordinate->points.push_back(coor);

        word >> x;
        coor.x = (atof(x.c_str()));
        word >> x;
        coor.y = (atof(x.c_str()));
        word >> x;
        coor.z = (atof(x.c_str()));
        coordinate->points.push_back(coor);

        word >> x;
        coor.x = (atof(x.c_str()));
        word >> x;
        coor.y = (atof(x.c_str()));
        word >> x;
        coor.z = (atof(x.c_str()));
        coordinate->points.push_back(coor);

        obj_num++;
        pcl::transformPointCloud(*coordinate, *coordinate, transform_2);

        x0 = coordinate->points[0].x;
        y0 = coordinate->points[0].y;
        z0 = coordinate->points[0].z;
        x1 = coordinate->points[1].x;
        y1 = coordinate->points[1].y;
        z1 = coordinate->points[1].z;
        x2 = coordinate->points[2].x;
        y2 = coordinate->points[2].y;
        z2 = coordinate->points[2].z;
        x3 = coordinate->points[3].x;
        y3 = coordinate->points[3].y;
        z3 = coordinate->points[3].z;
        x4 = coordinate->points[4].x;
        y4 = coordinate->points[4].y;
        z4 = coordinate->points[4].z;
        x5 = coordinate->points[5].x;
        y5 = coordinate->points[5].y;
        z5 = coordinate->points[5].z;
        x6 = coordinate->points[6].x;
        y6 = coordinate->points[6].y;
        z6 = coordinate->points[6].z;
        x7 = coordinate->points[7].x;
        y7 = coordinate->points[7].y;
        z7 = coordinate->points[7].z;
        gt_l.push_back(sqrt(pow((x0 - x3), 2) + pow((y0 - y3), 2)));
        gt_w.push_back(sqrt(pow((x0 - x1), 2) + pow((y0 - y1), 2)));
        gt_h.push_back(sqrt(pow((z0 - z4), 2)));

        gt_x.push_back((x0 + x1 + x2 + x3 + x4 + x5 + x6 + x7) / 8);
        gt_y.push_back((y0 + y1 + y2 + y3 + y4 + y5 + y6 + y7) / 8);
        gt_z.push_back((z0 + z1 + z2 + z3 + z4 + z5 + z6 + z7) / 8);

        float sina = (y0 - y3) / sqrt(pow((x0 - x3), 2) + pow((y0 - y3), 2));
        float cosa = (x0 - x3) / sqrt(pow((x0 - x3), 2) + pow((y0 - y3), 2));
        gt_yaw.push_back(atan2(sina, cosa));
    }
    in.close();
    ROS_INFO("current_frame:%s %d/%d   obj_num:%d", data_seq.c_str(), cur_num + 1, total_num, obj_num);
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::MarkerArray marker_array_text;
    visualization_msgs::Marker bbox_marker;
    visualization_msgs::Marker marker1;
    bbox_marker.header.frame_id = "origin";
    bbox_marker.header.stamp = ros::Time::now();
    bbox_marker.ns = "";
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 1.0f;
    bbox_marker.color.b = 1.0f;
    bbox_marker.color.a = 0.8f;
    bbox_marker.lifetime = ros::Duration(5);
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;

    marker1.header.frame_id = "origin";
    marker1.header.stamp = ros::Time::now();
    marker1.ns = "";
    marker1.lifetime = ros::Duration(5);
    marker1.action = visualization_msgs::Marker::ADD;
    marker1.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    for (size_t i = 0; i < gt_cls.size(); i++)
    {
        bbox_marker.id = i;
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

        marker1.id = i;

        marker1.color.r = 1.0f;
        marker1.color.g = 1.0f;
        marker1.color.b = 1.0f;

        marker1.color.a = 1.0f;
        marker1.scale.z = 2.0;

        marker1.pose.orientation.w = 1.0;
        marker1.pose.position.x = gt_x[i];
        marker1.pose.position.y = gt_y[i];
        marker1.pose.position.z = gt_z[i];

        marker1.text = ("<" + tracking_id[i] + ">" + gt_cls[i] + to_string(sqrt(gt_x[i] * gt_x[i] + gt_y[i] * gt_y[i] + gt_z[i] * gt_z[i])).substr(0, 5) + "m" + "\n").c_str();

        marker_array_text.markers.push_back(marker1);
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

            marker1.id = i;
            marker1.color.r = 1;
            marker1.color.g = 1;
            marker1.color.b = 1;

            marker1.color.a = 0;
            marker1.scale.z = 1;

            marker1.pose.orientation.w = 1.0;
            marker1.pose.position.x = 0;
            marker1.pose.position.y = 0;
            marker1.pose.position.z = 0;

            marker1.text = "aa";
            marker_array_text.markers.push_back(marker1);
        }
    }
    marker1.id = gt_cls.size();
    marker1.scale.z = 3.0;
    marker1.color.b = 0.0f;
    marker1.color.g = 0.0f;
    marker1.color.r = 1.0f;
    marker1.color.a = 1.0;
    marker1.lifetime = ros::Duration(float(1.0 / demo_frequence));
    marker1.pose.position.x = 0;
    marker1.pose.position.y = 0;
    marker1.pose.position.z = 1;
    marker1.text = ("current_frame:" + to_string(cur_num + 1) + "/" + to_string(total_num) + "\n").c_str();
    marker_array_text.markers.push_back(marker1);

    anno_pub.publish(marker_array);
    txt_pub.publish(marker_array_text);
    preBoxNum = gt_cls.size();
}
void image_read(string seq, string img_name, int lidar_num, string anno_name)
{
    string imgName = root_path + "/data/" + seq + "/image/" + to_string(lidar_num) + "/" + img_name;
    string img_anno = root_path + "/label/" + seq + "/image/" + to_string(lidar_num) + "/" + anno_name;
    cv::Mat image = cv::imread(imgName, CV_LOAD_IMAGE_COLOR);
    if (image.empty())
    {
        printf("open image error\n");
    }
    float l_x, l_y, r_x, r_y;
    char stat[200], visualable[200], cls[200];
    int id;
    fstream in(img_anno.c_str());
    string line;
    int obj_num = 0;
    int tm_n = 0;
    while (getline(in, line))
    {
        stringstream word(line);
        string x = "";
        word >> x;
        id = atoi(x.c_str());
        word >> x;
        strncpy(cls, x.c_str(), x.length() + 1);
        word >> x;
        strncpy(stat, x.c_str(), x.length() + 1);
        word >> x;
        strncpy(visualable, x.c_str(), x.length() + 1);
        word >> x;
        l_x = (atof(x.c_str()));
        word >> x;
        l_y = (atof(x.c_str()));
        word >> x;
        r_x = (atof(x.c_str()));
        word >> x;
        r_y = (atof(x.c_str()));
        tm_n++;
        cv::rectangle(image, Point(l_x, l_y), Point(r_x, r_y), Scalar(255, 255, 0), 3, LINE_8, 0);
    }
    in.close();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    img_pub.publish(msg);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "read_pub");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    img_pub = it.advertise("camera/image", 1);
    pcd_pub = n.advertise<sensor_msgs::PointCloud2>("pcd_single_pointcloud", 100);
    anno_pub = n.advertise<visualization_msgs::MarkerArray>("boxes", 100);
    txt_pub = n.advertise<visualization_msgs::MarkerArray>("txtes", 100);
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
        for (int lidar_num = 1; lidar_num < 7; lidar_num++)
        {
            vector<string> image_list;
            string img_pathlist = root_path + "/data/" + seqlist[i] + "/image/" + to_string(lidar_num) + ".txt";
            ifstream in1(img_pathlist);
            string line1;
            if (in1)
            {
                while (getline(in1, line1))
                {
                    image_list.push_back(line1);
                }
            }
            else
            {
                cout << "no such file" << endl;
            }
            for (int n = 0; n < pcd_list.size(); n++)
            {
                pcd_read(pcd_list[n], seqlist[i], lidar_num);
                string anno_name = pcd_list[n].substr(0, 13) + "txt";
                anno_read(anno_name, seqlist[i], n, pcd_list.size(), lidar_num);
                image_read(seqlist[i], image_list[n], lidar_num, anno_name);
                ros::spinOnce();
                loop_rate.sleep();
                if (!ros::ok())
                    return 0;
            }
        }
        pcd_list.clear();
    }
    ros::spin();

    return 0;
}