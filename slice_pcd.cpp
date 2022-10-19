#include "ros/ros.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>    
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/bearing_angle_image.h>
#include <pcl/range_image/range_image.h>   
#include <pcl/visualization/pcl_visualizer.h>

#include <sensor_msgs/PointCloud2.h>

#include<opencv2/highgui/highgui.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/face.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>

#include<time.h>
#include<iostream>
#include<stdlib.h>

static std::string PATH = "/home/cyppt/catkin_ws/src/try_pcd/map_to_slice.pcd";
#define INIT_SEED_NUM 10  //优化函数初始化种子的个数
#define MAX_ITER_NUM 20 // 优化函数 最大迭代次数

namespace pcl {
    struct PointXYZOR    /*包含xyz信息， 包含ODVL深度信息gray_ODVL,
    以及计算ODVL所需的：
    BA角度值gray_bearing_angle(0-255)，
    最优深度gray_optimal_depth(0-255)
    以及雷达线束ring*/ 
    {       
        float x;
        float y;
        float z;
        float gray_ODVL;
        int gray_bearing_angle;
        int gray_optimal_depth;
        int ring;
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(
    pcl::PointXYZOR ,    //注册点类型宏   
    (float ,x,x)    
    (float ,y,y)   
    (float ,z,z)   
    (float ,gray_ODVL,gray_ODVL)  
    (int ,gray_bearing_angle,gray_bearing_angle)
    (int ,gray_optimal_depth,gray_optimal_depth)
    (int ,ring,ring)
 )

struct Cseed
{
    double param1;
    double param2;
    double var;
    double gradient;
};

class Cslice_pcd
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pcl_pub_;
    ros::Subscriber point_cloud_sub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ordered_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZOR>::Ptr cloud_2d_ptr_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr check_pcd_ptr_;
    pcl::BearingAngleImage cloud_ba_ptr_;
    Eigen::Vector3f pc_main_direction_;
    sensor_msgs::PointCloud2 output_;
public:
    Cslice_pcd(ros::NodeHandle nh);
    void Pub_pcd();
    void CheckPointCloud(std::string pcd_name = "check");
    void BearingAngleImageOutput();
    void RangeImageOutput();
    void PointCloundPCA();
    void ProjectToHorizontalPlane();
    void ReceivePointCloud(const sensor_msgs::PointCloud2::ConstPtr &pc_ptr);
    void UnorderedPCToOrderedPC();
    int DetermineQuadrant(float x, float y);
    void OptimizationODVL();
    double GetRandNum();
    double GetRandNum(double range_min, double range_max);
    double CalculateVariance(double parameter1);
    double CalculateGradient(double parameter1);
    void CheckOrderedPC();
};

int cmp( const void *a ,const void *b);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slice_pcd");
    ros::NodeHandle nh;
    Cslice_pcd slice_pcd1(nh);

    ros::Rate loop_rate(1);
    slice_pcd1.PointCloundPCA();
    slice_pcd1.UnorderedPCToOrderedPC();
    slice_pcd1.CheckOrderedPC();
    slice_pcd1.ProjectToHorizontalPlane();
   slice_pcd1.OptimizationODVL();
    while (ros::ok())
    {
        slice_pcd1.Pub_pcd();
        ros::spinOnce();
        loop_rate.sleep();
        slice_pcd1.CheckOrderedPC();
    }
    // slice_pcd1.CheckPointCloud("map_to_slice");
    return 0;
}

Cslice_pcd::Cslice_pcd(ros::NodeHandle nh)
{
    nh_ = nh;
    cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_2d_ptr_.reset(new pcl::PointCloud<pcl::PointXYZOR>);
    check_pcd_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    ordered_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ordered_cloud_ptr_->width = 18300;
    ordered_cloud_ptr_->height = 75;
    ordered_cloud_ptr_->points.resize(ordered_cloud_ptr_->width * ordered_cloud_ptr_->height);

    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_output",  1);
    point_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, &Cslice_pcd::ReceivePointCloud,this);

    pcl::io::loadPCDFile(PATH, *cloud_ptr_);
    pcl::toROSMsg(*cloud_ptr_,output_);
    output_.header.frame_id = "map";  // fixed_fram ----rviz

    // pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>& pointCloud = *pointCloudPtr; //这个对象相当于是一个点集，存放着类型为pcl::PointXYZ的点
    
    // for(float y=-0.5f;y<= 0.5f; y+=0.01f)
    // {   
    //     for (float z=-0.5f;z<=0.5f;z+=0.01f)
    //     {
    //         pcl::PointXYZ point;                //依次创建若干点，并保存到点云点集中
    //         point.x =2.0f -y;
    //         point.y =y;
    //         point.z =z;
    //         pointCloud.points.push_back(point);    
    //     }
    // }

    // pointCloud.width  = (uint32_t) pointCloud.points.size();
    // pointCloud.height = 1;

    // cloud_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pointCloud);
}

void Cslice_pcd::Pub_pcd()
{
    pcl_pub_.publish(output_); 
}

void Cslice_pcd::CheckPointCloud(std::string pcd_name)
{
    //对于无序点云hight默认是1  不设置 height和width 无法输出
    // in_cloud_ptr->height = 1;
    // in_cloud_ptr->width = in_cloud_ptr->size();
    pcl::io::savePCDFileASCII("/home/cyppt/catkin_ws/src/try_pcd/" + pcd_name + ".pcd", *cloud_ptr_);
    std::cout << "已输出待查看点云至：" << "/home/cyppt/catkin_ws/src/try_pcd/" + pcd_name << std::endl;
}

void Cslice_pcd::BearingAngleImageOutput()
{
    cloud_ba_ptr_.generateBAImage(*ordered_cloud_ptr_);


        //保存BA图
    cv::Mat Im(cloud_ba_ptr_.height, cloud_ba_ptr_.width, CV_8UC1, cv::Scalar(255));
    int count = 0;
   std::cout<<"点云集1的宽度数："<<cloud_ba_ptr_.width <<std::endl;
   std::cout<<"点云集1的高度数："<<cloud_ba_ptr_.height  <<std::endl;
    for (int i=0; i< cloud_ba_ptr_.height; i++)
    {
        for (int j=0; j< cloud_ba_ptr_.width; j++)
        {
            Im.at<uchar>(i,j) = (cloud_ba_ptr_.points[count].rgba >> 8) & 0xff;
            count++;
        }
    }
    imwrite("/home/cyppt/catkin_ws/src/try_pcd/src_BA.jpg", Im);

}

void Cslice_pcd::RangeImageOutput()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>& pointCloud = *pointCloudPtr; //这个对象相当于是一个点集，存放着类型为pcl::PointXYZ的点
    
    for(float y=-0.5f;y<= 0.5f; y+=0.01f)
    {   
        for (float z=-0.5f;z<=0.5f;z+=0.01f)
        {
            pcl::PointXYZ point;                //依次创建若干点，并保存到点云点集中
            point.x =2.0f -y;
            point.y =y;
            point.z =z;
            pointCloud.points.push_back(point);    
        }
    }

    pointCloud.width  = (uint32_t) pointCloud.points.size();
    pointCloud.height = 1;

    /** 以下为从点云生成深度图的重要参数：
     *  -pointCloud：被检测点云
        -angularResolution=1：邻近的像素点所对应的每个光束之间相差 1°
        -maxAngleWidth=360：进行模拟的距离传感器对周围的环境拥有一个完整的360°视角，无论任何数据集都推荐使用此设置，因为最终获取的深度图像将被裁剪到有空间物体存在的区域范围
        -maxAngleHeight=180： 当传感器后面没有可以观测的点时，设置一个水平视角为180°的激光扫描仪即可，因为需要观察距离传感器前面就可以了。
        -sensorPose： 定义了模拟深度图像获取传感器的6DOF（6自由度）位置，其原始值为横滚角roll、俯仰角 pitch、偏航角 yaw 都为 0
        -coordinate_frame： 设置为CAMERA_FRAME说明系统的X轴是向右的、Y轴是向下的、Z轴是向前的，另外参数值是LASER_FRAME，其X轴向前、Y轴向左、Z轴向上
        -noiseLevel=0： 是指使用一个归一化的 Z缓存区来创建深度图像，如果想让邻近点集都落在同一个像素单元，可以设置一个较高的值，例如 noiseLevel = 0.05 可以理解为深度距离值是通过查询点半径为 5cm 的圆内包含的点用来平均计算而得到的 。
        -minRange=0：如果设置>0则所有模拟器所在位置半径 minRange 内的邻近点都将被忽略，即为盲区。
        -borderSize=1：如果设置>0 ,在裁剪图像时，将在图像周围留下当前视点不可见点的边界 。
     * 
     * **/
    float angularResolution = (float)(1.0f *(M_PI/180.0f));   //角分辨率
    float maxAngleWidth  = (float)(360.0f *(M_PI/180.0f));    //传感器视角360度
    float maxAngleHeight = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME; //！！！
    float noiseLevel=0.00;
    float minRange = 0.0f;
    int borderSize = 1;

    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage& rangeImage = *range_image_ptr;
 
    rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  
    std::cout << rangeImage << "\n";
    // --------------------------------------------
    // -----在3D 视窗中 添加显示点云-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer viewer("3D viewer"); //创建一个3D可视化界面
    viewer.setBackgroundColor (1,1,1); //设置视窗背景颜色rgb111
 
    // 添加深度图点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
    viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "range image");
 
   // 添加原始点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> org_image_color_handler (pointCloudPtr, 255, 100, 0);
    viewer.addPointCloud (pointCloudPtr, org_image_color_handler, "orginal image");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "orginal image");

    viewer.initCameraParameters();
    viewer.addCoordinateSystem(1.0);
 
    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
        pcl_sleep(0.01);
    }
}

void Cslice_pcd::PointCloundPCA()
{
    pcl::PointCloud<pcl::PointXYZ> center_normalize;
    double center_x = 0.0, center_y = 0.0, center_z = 0.0;
    Eigen::Matrix3f point_cloud_cov = Eigen::Matrix3f(); // 协方差矩阵
    Eigen::MatrixXf point_cloud_matrix = Eigen::MatrixXf();
    center_normalize = *cloud_ptr_;
    const int size_of_pointcloud = center_normalize.points.size();

    point_cloud_matrix.resize(3,size_of_pointcloud);

    //std::cout << "size of point cloud : "<< size_of_pointcloud << std::endl;
    //点云去中心化
    #pragma omp for // OpenMP并行化执行这条语句后的for循环，从而起到加速的效果
    for (size_t i = 0; i < center_normalize.points.size(); i ++)
    {
        center_x += center_normalize.at(i).x;
        center_y += center_normalize.at(i).y;
        center_z += center_normalize.at(i).z;
    }
    center_x /= center_normalize.points.size();
    center_y /= center_normalize.points.size();
    center_z /= center_normalize.points.size();

    // std::cout << center_x << std::endl;
    // std::cout << center_y << std::endl;
    // std::cout << center_z << std::endl;

    #pragma omp for // OpenMP并行化执行这条语句后的for循环，从而起到加速的效果
    for (size_t i = 0; i < center_normalize.points.size(); i ++)
    {
        center_normalize.at(i).x -= center_x;
        center_normalize.at(i).y -= center_y;
        center_normalize.at(i).z -= center_z;
        point_cloud_matrix(0,i) =  center_normalize.at(i).x;
        point_cloud_matrix(1,i) =  center_normalize.at(i).y;
        point_cloud_matrix(2,i) =  center_normalize.at(i).z;
    }

    //SVD分解
    point_cloud_cov = point_cloud_matrix * (point_cloud_matrix.transpose());
    Eigen::JacobiSVD< Eigen::MatrixXf> svd(point_cloud_cov,  Eigen::ComputeFullU |  Eigen::ComputeFullV);
    //std::cout << "Its singular values are : " << std::endl << svd.singularValues() << std::endl;
    //std::cout << svd.matrixU() << std::endl;

    pc_main_direction_ = svd.matrixU().col(0);
    pc_main_direction_ = pc_main_direction_ * 10;

    std::cout << "It's main direction is : " << std::endl << pc_main_direction_ << std::endl;
}

void Cslice_pcd::ProjectToHorizontalPlane()
{
    // BA角度变量
    double max_gray_optimal_depth = 0;
    double aver_gray_optimal_depth = 0; 
    double aver_ga_depth = 0;
    double var_go = 0, var_ga = 0;
    double theta = 0;
    // 最优深度变量
    double x, y, z;  // 计算最优深度，在main direction 上的点
    double max_left_x = 0x3f3f3f , max_low_y = 0x3f3f3f; // 左下边界
    float distance_a_b;
    float distance_a_c;
    float distance_b_c;
    float cos_A;
    float sin_A;
    // 线束变量
    float angle;
    int scanID;
    static int RING_ID_MAP_16[] = {0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8};  // 速腾雷达线束

    x = pc_main_direction_.x();
    y = pc_main_direction_.y();
    z = pc_main_direction_.z();
    
    // 寻找点云 左下边界
    for (int i = 0; i < ordered_cloud_ptr_->height; i ++)
    {
        for (int j = 0; j < ordered_cloud_ptr_->width; j++)
        {
            if (ordered_cloud_ptr_->at(j,i).x < max_left_x) max_left_x = ordered_cloud_ptr_->at(j,i).x;
            if (ordered_cloud_ptr_->at(j,i).y < max_low_y) max_low_y = ordered_cloud_ptr_->at(j,i).y;
        }
    }

    std::cout << "max_left_x :" << max_left_x << "\nmax_low_y :" << max_low_y << "\n";
    x += max_left_x;
    y += max_low_y;

    for (int i = 0; i < ordered_cloud_ptr_->height; i ++)
    {
        for (int j = 0; j < ordered_cloud_ptr_->width; j++)
        {
            // 激光雷达线束 未用到
            angle = atan(ordered_cloud_ptr_->at(j,i).z / sqrt(pow(ordered_cloud_ptr_->at(j,i).x, 2) + pow(ordered_cloud_ptr_->at(j,i).y,2))) * 180 / M_PI; //计算激光雷达点倾斜角度，按照角度进行分类
            scanID = int((angle + 15) / 2 + 0.5);
            pcl::PointXYZOR point;
            if (scanID > 15) scanID = 15;
            else if (scanID < 0 ) scanID = 0;
            point.ring = RING_ID_MAP_16[scanID];

            // 投影到水平面
            point.x = ordered_cloud_ptr_->at(j,i).x;
            point.y = ordered_cloud_ptr_->at(j,i).y;
            point.z = 0;
            point.gray_ODVL = 0;
            // 计算深度信息 
            // bearing angle
            if ( i < ordered_cloud_ptr_->height -1 && j < ordered_cloud_ptr_->width - 1)
            {
                // 参考 github pcl 中bearing angle 的源码
                theta =  cloud_ba_ptr_.getAngle(ordered_cloud_ptr_->at(j,i + 1), ordered_cloud_ptr_->at(j + 1,i));
                point.gray_bearing_angle = theta * 255 / 180;
            }
            else if (i == ordered_cloud_ptr_->height -1 && j < ordered_cloud_ptr_->width - 1)
            {
                theta =  cloud_ba_ptr_.getAngle(ordered_cloud_ptr_->at(j,i - 1), ordered_cloud_ptr_->at(j + 1,i));
                point.gray_bearing_angle = theta * 255 / 180;
            }
            else if (i < ordered_cloud_ptr_->height -1 && j == ordered_cloud_ptr_->width - 1)
            {
                theta =  cloud_ba_ptr_.getAngle(ordered_cloud_ptr_->at(j - 1,i + 1), ordered_cloud_ptr_->at(j,i));
                point.gray_bearing_angle = theta * 255 / 180;
            }
             else 
            {
                theta =  cloud_ba_ptr_.getAngle(ordered_cloud_ptr_->at(j - 1,i - 1), ordered_cloud_ptr_->at(j,i));
                point.gray_bearing_angle = theta * 255 / 180;
            }
             if ( point.gray_bearing_angle < 0)
             {
                //std::cout << "check theta:" << theta << "\n";
                //std:: cout << "j(width) i(height) :" << j << " , " << i << "\n"; 
                point.gray_bearing_angle = cloud_2d_ptr_->points[i * ordered_cloud_ptr_->width + j - 1].gray_bearing_angle;
                // BA::getAngle 三边不构成三角形 会返回 nan 是 0度
             }

            // optimal_depth 未标准化到0-255
            // a 点云左下边界 b 过a点且在main direction上的点 c 平面上的点  构成三角形
            distance_a_b = sqrt(pow(max_left_x - x, 2) + pow(max_low_y - y, 2) + pow(0 - z, 2));
            distance_a_c = sqrt(pow(max_left_x - point.x, 2) + pow(max_low_y - point.y, 2) + pow(0 -point.z, 2));
            distance_b_c = sqrt(pow(point.x - x, 2) + pow(point.y - y, 2) + pow(point.z - z, 2));
            cos_A = (pow(distance_a_b, 2) + pow(distance_a_c, 2) - pow(distance_b_c, 2))/(2 * distance_a_b * distance_a_c);
            sin_A = sqrt(1 - pow(cos_A , 2));

            if (distance_a_c != 0) point.gray_ODVL = distance_a_c * sin_A; // 先将未 标准化的gray_optimal_depth 存在这里
            else point.gray_ODVL = 0; // distance_a_c == 0 则 不够成三角形 a 与 c 重合 距离为0
            aver_gray_optimal_depth += point.gray_ODVL;
            if (point.gray_ODVL < 0) std::cout <<"check error gd:" << point.gray_ODVL << "\n";
            if (point.gray_ODVL > max_gray_optimal_depth) max_gray_optimal_depth = point.gray_ODVL;
            //aver_gray_optimal_depth += point.gray_ODVL;
            // if (j % 100 == 0)
            // {
            //     std::cout <<"check gb:" << point.gray_bearing_angle << "\n";
            // }
            if (aver_gray_optimal_depth <=0)
            {
                std::cout <<"check sum gd:" << aver_gray_optimal_depth << "\n";
                std::cout <<"check gd:" << point.gray_ODVL << "\n";
                std::cout <<"check distance_a_c" << distance_a_c << "\n";
                std::cout <<"check distance_a_b" << distance_a_b << "\n";
                std::cout <<"check distance_b_c" << distance_b_c << "\n";
                std::cout <<"cos_A " << cos_A  << "\n";
                std::cout <<"sin_A" << sin_A << "\n";
                aver_gray_optimal_depth = 0;
            }
            point.gray_optimal_depth = 0;
            cloud_2d_ptr_->push_back(point);
            //std::cout << "ring:"<<scanID << std::endl;
        }
    }

    std::cout <<"sum gd:" << aver_gray_optimal_depth << "\n";
    aver_gray_optimal_depth /= cloud_2d_ptr_->size();
    std::cout << "size of 2d cloud :" << cloud_2d_ptr_->size() << "\n";
    std::cout <<"max gd:" << max_gray_optimal_depth << "\n";
    std::cout <<"aver gd:" << aver_gray_optimal_depth << "\n";

    // 继续计算optimal_depth    使其标准化到0-255
    #pragma omp for // OpenMP并行化执行这条语句后的for循环，从而起到加速的效果
    for (int i = 0; i < cloud_2d_ptr_->size(); i++)
    {
        // if (i % 100 == 0)
        // {
        //     std ::cout << "check gb :" << cloud_2d_ptr_->points[i].gray_bearing_angle << "\ncheck gd:" << cloud_2d_ptr_->points[i].gray_optimal_depth << "\n";
        //     std::cout <<"gd:" << cloud_2d_ptr_->points[i].gray_ODVL << "\n";
        // }
        cloud_2d_ptr_->points[i].gray_optimal_depth = cloud_2d_ptr_->points[i].gray_ODVL / max_gray_optimal_depth * 255;
        if ( cloud_2d_ptr_->points[i].gray_optimal_depth > 255) cloud_2d_ptr_->points[i].gray_optimal_depth = 255;
        else if ( cloud_2d_ptr_->points[i].gray_optimal_depth < 0)  cloud_2d_ptr_->points[i].gray_optimal_depth = 0;
        cloud_2d_ptr_->points[i].gray_ODVL = 0;

        aver_ga_depth += cloud_2d_ptr_->points[i].gray_bearing_angle;
        if ( cloud_2d_ptr_->points[i].gray_bearing_angle < 0) cout << "error ba:" <<  cloud_2d_ptr_->points[i].gray_bearing_angle << "\n";
    }
    std::cout <<"sum ga:" << aver_ga_depth << "\n";
    aver_ga_depth /= cloud_2d_ptr_->size();
    std::cout <<"aver ga:" << aver_ga_depth << "\n";
    // 计算方差
    for (int i = 0; i < cloud_2d_ptr_->size(); i++)
    {
        var_ga += pow(cloud_2d_ptr_->points[i].gray_bearing_angle - aver_ga_depth, 2);
        var_go += pow(cloud_2d_ptr_->points[i].gray_optimal_depth - aver_gray_optimal_depth , 2);
    }
    var_ga /= cloud_2d_ptr_->size();
    var_go /= cloud_2d_ptr_->size();
    std::cout <<"var ga:" << var_ga << "\n";
    std::cout <<"var go:" << var_go << "\n";
    
}

void Cslice_pcd::ReceivePointCloud(const sensor_msgs::PointCloud2::ConstPtr &pc_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc_ptr, *temp_cloud);
    std::cout<<"点云集的宽度数："<<temp_cloud->width <<std::endl;
    std::cout<<"点云集的高度数："<<temp_cloud->height  <<std::endl;
    *cloud_ptr_ += (*temp_cloud);
    ROS_INFO("sum received %ld points", cloud_ptr_->points.size());
}

void Cslice_pcd::UnorderedPCToOrderedPC()
{
    int quadrant_indicator = 0; //  0-3 第一象限 ～ 第四象限
    int last_quadrant_indicator = 0;
    int quadrant_change_count = 0;
    int init_quadrant = 0;
    double theta = 0.0, init_theta = 0.0;
    int count = 0;
    int width_count = 0, height_count = 0;
    int min_width = 0x3f3f3f; // 用于对齐
    double aver_width = 0;
    auto temp_x = cloud_ptr_->points[0].x;
    auto temp_y = cloud_ptr_->points[0].y;

    theta = atan(temp_y / temp_x); // 激光雷达顺时针扫描，在象限内，theta 数值会越来越小
    init_theta = theta;
    init_quadrant = DetermineQuadrant(temp_x, temp_y);
    quadrant_indicator = init_quadrant;

    for (count = 0 ; count < cloud_ptr_->size(); count++)
    {
        auto temp_x = cloud_ptr_->points[count].x;
        auto temp_y = cloud_ptr_->points[count].y;
        theta = atan(temp_y / temp_x);
        //if(count % 1000 == 0) std::cout << "x:" << temp_x << "\ny:" << temp_y << "\ntheta:" << theta << std::endl;
        last_quadrant_indicator = quadrant_indicator;
        quadrant_indicator = DetermineQuadrant(temp_x, temp_y);
        // //按照旋转角度划分点云层数 
        if (last_quadrant_indicator != quadrant_indicator) quadrant_change_count++;
        if (quadrant_change_count == 4)
        {
            if (theta <= init_theta)
            {
                quadrant_change_count = 0;
                //std::cout << "height:" << height_count << "\nwidth:" << width_count << "\n";
                double temp_aver;
                if (height_count == 0 ) temp_aver = width_count; // 程序第一次运行时的平均值
                else temp_aver  = aver_width / height_count; // 不包括当前的平均值， 不用给height+1
                //std::cout << "temp aver:" << temp_aver << "\n";
                if (abs(width_count - temp_aver) <  1000)  // 去除数据量过少的层数
                {
                    if (width_count < min_width) min_width = width_count;
                    aver_width += width_count;
                    height_count++;
                }
                //else std::cout << "Abort this layer point cloud!\n";
                width_count = 0;
                //std::cout << "x:" << temp_x << "\ny:" << temp_y << "\ntheta:" << theta << std::endl;
            }
        }
        
        ordered_cloud_ptr_->at(width_count,height_count).x = cloud_ptr_->points[count].x;
        ordered_cloud_ptr_->at(width_count,height_count).y = cloud_ptr_->points[count].y;
        ordered_cloud_ptr_->at(width_count,height_count).z = cloud_ptr_->points[count].z;
        width_count++;
    }
    ordered_cloud_ptr_->width = min_width + 1;
    ordered_cloud_ptr_->height = height_count;
    // aver_width /= (height_count - 1);
    // cout << "aver width:" << aver_width << "\n";
    cout << "orderer cloud width:" <<ordered_cloud_ptr_->width << "\norderer cloud height:" << ordered_cloud_ptr_->height << "\n";
}

int Cslice_pcd::DetermineQuadrant(float x, float y)
{
    if (x > 0 && y >=0) return 0;
    else if (x >= 0 && y < 0) return 1;
    else if (x < 0 && y <= 0) return 2;
    else if (x <= 0 && y > 0) return 3;
    else return -1;
}

void Cslice_pcd::OptimizationODVL()
{
    Cseed init_seed[INIT_SEED_NUM];
    Cseed select_seed[INIT_SEED_NUM * 2];
    double max_var = 0, max_param1;
    for (int i = 0; i < INIT_SEED_NUM; i++)
    {
        init_seed[i].param1 = GetRandNum();
        //std::cout << "check param1 :" << init_seed[i].param1 << "\n";
        init_seed[i].param2 = 1 - init_seed[i].param1;
        init_seed[i].var = CalculateVariance(init_seed[i].param1);
        //std::cout << "check var :" << init_seed[i].var << "\n";
        init_seed[i].gradient = CalculateGradient(init_seed[i].param1);
        //std::cout << "check gradient :" << init_seed[i].gradient << "\n";
    }
    for (int i = 0; i < MAX_ITER_NUM; i++)
    {
        for (int j = 0; j < INIT_SEED_NUM; j++)
        {
            select_seed[j] = init_seed[j];
            if (init_seed->gradient > 0)
            {
                select_seed[j + INIT_SEED_NUM].param1 = init_seed[j].param1 + GetRandNum(0.01,0.2);
                if (select_seed[j + INIT_SEED_NUM].param1 >= 1) select_seed[j + INIT_SEED_NUM].param1 = 0.9999;
                else if (select_seed[j + INIT_SEED_NUM].param1 <= 0) select_seed[j + INIT_SEED_NUM].param1 = 0.0001;
            }
            else
            {
                select_seed[j + INIT_SEED_NUM].param1 = init_seed[j].param1 - GetRandNum(0.01,0.2);
                if (select_seed[j + INIT_SEED_NUM].param1 >= 1) select_seed[j + INIT_SEED_NUM].param1 = 0.9999;
                else if (select_seed[j + INIT_SEED_NUM].param1 <= 0) select_seed[j + INIT_SEED_NUM].param1 = 0.0001;
            }
            select_seed[j + INIT_SEED_NUM].param2 = 1 - select_seed[j + INIT_SEED_NUM].param1;
            select_seed[j + INIT_SEED_NUM].var = CalculateVariance(select_seed[j + INIT_SEED_NUM].param1);
            select_seed[j + INIT_SEED_NUM].gradient = CalculateGradient(select_seed[j + INIT_SEED_NUM].param1);
        }
        qsort(select_seed, INIT_SEED_NUM * 2, sizeof(select_seed[0]), cmp);
        for (int j = 0; j < INIT_SEED_NUM; j++)
        {
            init_seed[j] = select_seed[j];
            //std::cout << "variance :" << init_seed[j].var << "\n";
        }
        if (init_seed[0].var > max_var)
        {
            max_var = init_seed[0].var;
            max_param1 = init_seed[0].param1;
        }
        std::cout << "iter times :" << i + 1  << "\nmax var now:" << init_seed[0].var << "\nparam1 now: " << init_seed[0].param1 << "\n";
    }
}

double Cslice_pcd::GetRandNum()
{
    srand((unsigned)time(NULL));
    return rand() / double(RAND_MAX);
}

double Cslice_pcd::GetRandNum(double range_min, double range_max)
{
    srand((unsigned)time(NULL));
    double temp = rand() / double(RAND_MAX);
    temp *= abs(range_max - range_min);
    temp += range_min;
    return temp;
}

double Cslice_pcd::CalculateVariance(double parameter1)
{
    double average = 0, variance = 0;
    double parameter2 = 1 - parameter1;
    
    #pragma omp for // OpenMP并行化执行这条语句后的for循环，从而起到加速的效果
    for (int i = 0; i < cloud_2d_ptr_->size(); i++)
    {
        cloud_2d_ptr_->points[i].gray_ODVL = (parameter1 * cloud_2d_ptr_->points[i].gray_bearing_angle + 
        parameter2 * cloud_2d_ptr_->points[i].gray_optimal_depth) / (parameter1 + parameter2);
        average += cloud_2d_ptr_->points[i].gray_ODVL;
    }
    average /= cloud_2d_ptr_->size();
    //std :: cout << "check average ODVL:" << average << "\n";
    #pragma omp for // OpenMP并行化执行这条语句后的for循环，从而起到加速的效果
    for (int i = 0; i < cloud_2d_ptr_->size(); i++)
    {
        variance += pow(cloud_2d_ptr_->points[i].gray_ODVL - average, 2);
    }
    variance /= cloud_2d_ptr_->size();
    return variance;
}

double Cslice_pcd::CalculateGradient(double parameter1)
{
    double k = parameter1 / (1- parameter1);
    double parameter2 = 1 - parameter1;
    double aver_GB = 0, aver_GD = 0;
    double gradient;
    double k1, k2;

    k1 = k / pow(k + 1, 3);
    k2 = 1 / pow(k + 1, 3);
    
    #pragma omp for // OpenMP并行化执行这条语句后的for循环，从而起到加速的效果
    for (int i = 0; i < cloud_2d_ptr_->size(); i++)
    {
        cloud_2d_ptr_->points[i].gray_ODVL = (parameter1 * cloud_2d_ptr_->points[i].gray_bearing_angle + 
        parameter2 * cloud_2d_ptr_->points[i].gray_optimal_depth) / (parameter1 + parameter2);
        aver_GB += cloud_2d_ptr_->points[i].gray_bearing_angle;
        aver_GD += cloud_2d_ptr_->points[i].gray_optimal_depth;
    }
    aver_GB /= cloud_2d_ptr_->size();
    aver_GD /= cloud_2d_ptr_->size();
    //std :: cout << "check average GB:" << aver_GB << "\n";
    //std :: cout << "check average GD:" << aver_GD << "\n";
    #pragma omp for // OpenMP并行化执行这条语句后的for循环，从而起到加速的效果
    for (int i = 0; i < cloud_2d_ptr_->size(); i++)
    {
        double temp = cloud_2d_ptr_->points[i].gray_bearing_angle - cloud_2d_ptr_->points[i].gray_optimal_depth - aver_GB + aver_GD;
        gradient = k1 * (cloud_2d_ptr_->points[i].gray_bearing_angle - aver_GB) * temp;
        gradient += k2 * (cloud_2d_ptr_->points[i].gray_optimal_depth - aver_GD) * temp;
        gradient *= 2;
    }
    gradient /= cloud_2d_ptr_->size();
    return gradient;
}

int cmp( const void *a ,const void *b) // 从大到小
{
    return (*(Cseed *)a).var > (*(Cseed *)b).var? -1 : 1; 
}

void Cslice_pcd::CheckOrderedPC()
{
    int count = 0;
    pcl::PointXYZI point;
    std::string pcd_name = "check";

    for (int i = 0; i < ordered_cloud_ptr_->height; i ++)
    {
        for (int j = 0; j < ordered_cloud_ptr_->width; j++)
        {
            point.x = ordered_cloud_ptr_->at(j,i).x;
            point.y = ordered_cloud_ptr_->at(j,i).y;
            point.z = ordered_cloud_ptr_->at(j,i).z;
            point.intensity = (i +  1) * 30;
            check_pcd_ptr_->push_back(point);
        }
    }
    pcl::toROSMsg(*check_pcd_ptr_,output_);
    output_.header.frame_id = "map";  // fixed_fram ----rviz
    // pcl::io::savePCDFileASCII("/home/cyppt/catkin_ws/src/try_pcd/" + pcd_name + ".pcd", *check_pcd_ptr_);
    // std::cout << "已输出待查看点云至：" << "/home/cyppt/catkin_ws/src/try_pcd/" + pcd_name << std::endl;
    Pub_pcd();
}







