#include "ros/ros.h"
#include "../matplotlibcpp.h" // 网上的绘图库

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Jacobi>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <sensor_msgs/PointCloud2.h>

#include <gps_common/conversions.h>

#include <iostream>
#include <algorithm>
#include <numeric>
#include <cmath>

const double PI = 3.1415926535;
const std::string NODENAME = "add_pcd";

class Cadd_pcd
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pcl_pub_clouda_;
    ros::Publisher pcl_pub_cloudb_;
    ros::Publisher pcl_pub_cloudout_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_seeds_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr not_ground_cloud_ptr_;
    sensor_msgs::PointCloud2 ros_clouda_;
    sensor_msgs::PointCloud2 ros_cloudb_;
    sensor_msgs::PointCloud2 ros_cloudout_;

    Eigen::Matrix<float, 3,  1> estimate_plane_coefficient_;  //去除地面函数中，所估计平面的系数 a b c
    Eigen::Matrix4f gps_init_transf_;   //  GPS初始变换
    Eigen::Matrix4f icp_transf_;              // ICP 匹配后的变换
    double estimate_plane_constant_;  // 去除地面函数中，所估计平面的常数 

    int ground_filter_init_select_num_; // 去除地面函数中， 第一次估计平面时 选取的参考点的个数
    int estimate_plane_iter_max_;            // 去除地面函数中， 最大迭代次数
    double ground_seeds_threshold_;     // 去除地面函数中  第一次估计平面时 判定为平面点的容忍度 单位米
    double ground_distance_threshold_;// 去除地面函数中  之后估计平面时 判定为平面点的容忍度 单位米
    double sensor_height_;                             // 用于滤掉z轴 过低的点云

    // GPS
    double latitude_a_, longitude_a_, altitude_a_;
    double latitude_b_, longitude_b_, altitude_b_;
    // 文件路径
    std::string path_a_, path_b_;
    // 用于画图展示
    std::vector<double> plot_x_icp, plot_y_icp,plot_x_gps, plot_y_gps;

public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr clouda_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudb_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudb_after_icp_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr clouda_ptr_copy_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudb_ptr_copy_;

    Cadd_pcd(ros::NodeHandle nh);
    void PubPcd();
    void DownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr);
    void ClipCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr);
    void PlaneGroundFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr);
    void ExtractInitialSeeds(pcl::PointCloud<pcl::PointXYZ>::Ptr in_sorted_cloud_ptr);
    void EstimatePlane();
    void CheckPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, std::string pcd_name = "check_pcd.pcd");
    void GPSInitTransf();
    void IcpGetTransf();
    void GicpGetTransf();
    void CheckFuseResult(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, std::string plot_name = "icp");
    void ComparisonFuse();
    void FusePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, std::string pcd_name = "check_pcd.pcd");
};

int main(int argc, char **argv)
{
    double pause_variable;
    ros::init(argc, argv, NODENAME);
    ros::NodeHandle nh;
    Cadd_pcd add_pcd1(nh);
    // add_pcd1.FusePcd();
    std::cout << "点云a去除地面：\n";
    add_pcd1.PlaneGroundFilter(add_pcd1.clouda_ptr_);
    std::cout << "点云b去除地面：\n";
    add_pcd1.PlaneGroundFilter(add_pcd1.cloudb_ptr_);
    add_pcd1.GPSInitTransf();
    add_pcd1.GicpGetTransf();
    add_pcd1.ComparisonFuse();
    // ros::Rate loop_rate(1);
    // while (ros::ok())
    // {
    //     add_pcd1.PubPcd();
    //     ros::spinOnce();
    //     std::cout << "通过cin 暂停 输入后继续输出点云\n";
    //     std::cin >> pause_variable;
    //     loop_rate.sleep();
    // }
    // std::cout << "通过cin 暂停 输入后继续输出点云\n";
    // std::cin >> pause_variable;
    return 0;
}

Cadd_pcd::Cadd_pcd(ros::NodeHandle nh)
{
    nh_ = nh;
    clouda_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloudb_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloudout_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ground_seeds_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ground_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    not_ground_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    pcl_pub_clouda_ = nh_.advertise<sensor_msgs::PointCloud2> ("clouda",  1);
    pcl_pub_cloudb_ = nh_.advertise<sensor_msgs::PointCloud2> ("cloudb",  1);
    pcl_pub_cloudout_ = nh_.advertise<sensor_msgs::PointCloud2> ("cloudout",  1);

    // 参数初始化  我不太会设置
    nh_.getParam( NODENAME + "/ground_filter_init_select_num", ground_filter_init_select_num_);
    nh_.getParam( NODENAME + "/estimate_plane_iter_max", estimate_plane_iter_max_);
    nh_.getParam( NODENAME + "/ground_seeds_threshold", ground_seeds_threshold_);
    nh_.getParam( NODENAME + "/ground_distance_threshold", ground_distance_threshold_);
    nh_.getParam( NODENAME + "/sensor_height", sensor_height_);
    // 输入路径和初始GPS信息
    nh_.getParam( NODENAME + "/path_a", path_a_);
    nh_.getParam( NODENAME + "/latitude_a", latitude_a_);
    nh_.getParam( NODENAME + "/longitude_a", longitude_a_);
    nh_.getParam( NODENAME + "/altitude_a", altitude_a_);
    nh_.getParam( NODENAME + "/path_b", path_b_);
    nh_.getParam( NODENAME + "/latitude_b", latitude_b_);
    nh_.getParam( NODENAME + "/longitude_b", longitude_b_);
    nh_.getParam( NODENAME + "/altitude_b", altitude_b_);
    
    // std::cout << path_a_ << std::endl;
    pcl::io::loadPCDFile<pcl::PointXYZ> (path_a_, *clouda_ptr_);
    auto temp = *clouda_ptr_;
    clouda_ptr_copy_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(temp);
    pcl::io::loadPCDFile<pcl::PointXYZ> (path_b_, *cloudb_ptr_);
    temp = *cloudb_ptr_;
    cloudb_ptr_copy_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(temp);
}

// ROS 输出 未用到
void Cadd_pcd::PubPcd()
{
    *cloudout_ptr_ = *clouda_ptr_ + *cloudb_ptr_;

    pcl::toROSMsg (*clouda_ptr_, ros_clouda_);
    pcl::toROSMsg (*cloudb_ptr_, ros_cloudb_);
    pcl::toROSMsg (*cloudout_ptr_, ros_cloudout_);

    auto temp_ros_time = ros::Time::now();
    ros_clouda_.header.stamp = temp_ros_time;
    ros_clouda_.header.frame_id = "map";
    ros_cloudb_.header.stamp = temp_ros_time;
    ros_cloudb_.header.frame_id = "map";
    ros_cloudout_.header.stamp = temp_ros_time;
    ros_cloudout_.header.frame_id = "map";
    
    ros_clouda_.header.frame_id = "map";
    pcl_pub_clouda_.publish(ros_clouda_);
    pcl_pub_cloudb_.publish(ros_cloudb_);
    pcl_pub_cloudout_.publish(ros_cloudout_);
}

// 降采样
void Cadd_pcd::DownSample(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;

    voxel_grid.setInputCloud(in_cloud_ptr);
    voxel_grid.setLeafSize(1.0f,1.0f,1.0f);
    voxel_grid.filter(*out_cloud_ptr);
}

// 去除错误点云(z数据过低)
void Cadd_pcd::ClipCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
    pcl::ExtractIndices<pcl::PointXYZ> cliper;
    cliper.setInputCloud(in_cloud_ptr);
    pcl::PointIndices indices;
    double clip_low = -0.8 * sensor_height_;  

    #pragma omp for // OpenMP并行化执行这条语句后的for循环，从而起到加速的效果
    for (size_t i = 0; i < in_cloud_ptr->points.size(); i ++)
    {
        if (in_cloud_ptr->points[i].z < clip_low) 
        {
            indices.indices.push_back(i); // 标记
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); // 去除被标记的点
    cliper.filter(*out_cloud_ptr);
}

//去除地面点云  https://blog.csdn.net/qq_38167930/article/details/119165988  Ground Plane Fitting
void Cadd_pcd::PlaneGroundFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr)
{
    DownSample(in_cloud_ptr,in_cloud_ptr);
    ClipCloud(in_cloud_ptr,in_cloud_ptr);
    std::sort(in_cloud_ptr->begin(), in_cloud_ptr->end(),[](const  pcl::PointXYZ &a,  pcl::PointXYZ &b){return a.z < b.z;});
    ExtractInitialSeeds(in_cloud_ptr);
    // ExtractInitialSeeds(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud_in)); // pcl 传递地址注意
    
    ground_cloud_ptr_ = ground_seeds_cloud_ptr_;
    for (int i = 0; i < estimate_plane_iter_max_; i ++)
    {
        EstimatePlane();
        ground_cloud_ptr_->clear();
        not_ground_cloud_ptr_->clear();
        //点云数据转换为矩阵存储 n*3维度表示
        Eigen::MatrixXf cloud_matrix(in_cloud_ptr->points.size(),3);
        int j = 0;
        for (auto p : in_cloud_ptr->points)
        {
            cloud_matrix.row(j++) << p.x, p.y, p.z;
        }
        //所有点与地平面法线的点乘，得到与地平面的距离
        Eigen::VectorXf distance_ground = cloud_matrix * estimate_plane_coefficient_;
        for (int j = 0; j < distance_ground.rows(); j++)
        {
            if (distance_ground[j] < ground_distance_threshold_ - estimate_plane_constant_)
            {
                ground_cloud_ptr_->points.push_back(in_cloud_ptr->points[j]);
            }
            else
            {
                not_ground_cloud_ptr_->points.push_back(in_cloud_ptr->points[j]);
            }
        }
        std::cout << "迭代次数: " << i + 1 << "\\" << estimate_plane_iter_max_ << '\r' << std::flush;
    }
    std::cout << "迭代完成\n";
    // CheckPointCloud(not_ground_cloud_ptr_);
}

// 去除地面点云——初始平面选取函数
void Cadd_pcd::ExtractInitialSeeds(pcl::PointCloud<pcl::PointXYZ>::Ptr in_sorted_cloud_ptr)
{
    double sum = 0;
    int count = 0;
    ground_filter_init_select_num_ = in_sorted_cloud_ptr->points.size() / 100;
    
    // 选取最小的ground_filter_init_select_num_个点

    #pragma omp for // OpenMP并行化执行这条语句后的for循环，从而起到加速的效果
    for (size_t i = 0; i < in_sorted_cloud_ptr->points.size() && count < ground_filter_init_select_num_; i++)
    {
        sum += in_sorted_cloud_ptr->points[i].z;
        count++;
    }

    double low_point_representative_height;
    if (count != 0) low_point_representative_height = sum / count;
    else low_point_representative_height = 0;

    ground_seeds_cloud_ptr_->clear();

    #pragma omp for // OpenMP并行化执行这条语句后的for循环，从而起到加速的效果
    for (size_t i = 0; i < in_sorted_cloud_ptr->points.size();i++)
    {
        if (in_sorted_cloud_ptr->points[i].z < low_point_representative_height + ground_seeds_threshold_)
        {
            ground_seeds_cloud_ptr_->points.push_back(in_sorted_cloud_ptr->points[i]);
            // std::cout << in_sorted_cloud_ptr->points[i].z << std::endl;
        }
    }
}

// 去除地面点云——拟合平面函数
void Cadd_pcd::EstimatePlane()
{
    Eigen::Matrix3f cloud_cov;
    Eigen::Vector4f cloud_mean;
    Eigen::Vector3f seeds_mean;

    pcl::computeMeanAndCovarianceMatrix(*ground_cloud_ptr_, cloud_cov, cloud_mean);
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cloud_cov, Eigen::DecompositionOptions::ComputeFullU);
    estimate_plane_coefficient_ = svd.matrixU().col(2);
    seeds_mean = cloud_mean.head<3>();
    estimate_plane_constant_ = - (estimate_plane_coefficient_.transpose() *  seeds_mean)(0,0);
}

// 检查中间结果和输出
void Cadd_pcd::CheckPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, std::string pcd_name)
{
    //对于无序点云hight默认是1  不设置 height和width 无法输出
    in_cloud_ptr->height = 1;
    in_cloud_ptr->width = in_cloud_ptr->size();
    pcl::io::savePCDFileASCII("/home/cyppt/catkin_ws/src/try_pcd/" + pcd_name, *in_cloud_ptr);
    std::cout << "已输出待查看点云至：" << "/home/cyppt/catkin_ws/src/try_pcd/" + pcd_name << std::endl;
}

//GPS 获得拼接地图初始变换关系
void Cadd_pcd::GPSInitTransf()
{
    double utmx_b, utmy_b, utmx_a, utmy_a;
    std::string utm_zonea, utm_zoneb;

    gps_common::LLtoUTM(latitude_a_, longitude_a_, utmy_a, utmx_a, utm_zonea);
    gps_common::LLtoUTM(latitude_b_, longitude_b_, utmy_b, utmx_b, utm_zoneb);

    double x = -utmx_a + utmx_b;
    double y = -utmy_a + utmy_b;
    double z = -altitude_a_ + altitude_b_;

    // utm 和 地图坐标 旋转关系
    double rad = 90.0 * PI / 180;
    Eigen::Vector3f trans_vec(x * cos(rad) + y * sin(rad), -x*sin(rad) + y * cos(rad), z);
    Eigen::Quaternionf quat_rot(1,0,0,0);

    gps_init_transf_.setIdentity();
    gps_init_transf_.block<3,1>(0,3) = trans_vec;
    gps_init_transf_.block<3,3>(0,0) = quat_rot.normalized().toRotationMatrix();
    // std::cout << "check:\n" << gps_init_transf_ << std::endl;
    pcl::transformPointCloud(*cloudb_ptr_,*cloudb_ptr_,gps_init_transf_);
}

// ICP匹配 获取更好的地图融合信息  舍弃 采用GICP
void Cadd_pcd::IcpGetTransf()
{
    pcl::PointCloud<pcl::PointXYZ> temp;
    temp = *clouda_ptr_;
    // *cloudout_ptr_ = *clouda_ptr_ + *cloudb_ptr_;
    // CheckPointCloud(cloudout_ptr_, "gps_out_pcd.pcd");
    std::cout << "GPS信息直接融合：\n";
    CheckFuseResult(clouda_ptr_, cloudb_ptr_, "gps");
    FusePointCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(temp), cloudb_ptr_, "gps_out_pcd.pcd");

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputTarget(clouda_ptr_);
    icp.setInputSource(cloudb_ptr_);

    pcl::PointCloud<pcl::PointXYZ> output_ans;
    icp.align(output_ans);
    cloudb_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(output_ans);

    std::cout<< " score: " << icp.getFitnessScore() << std::endl;
    
    icp_transf_ = icp.getFinalTransformation();

    // *cloudout_ptr_ = *clouda_ptr_ + *cloudb_ptr_;
    // CheckPointCloud(cloudout_ptr_, "icp_out_pcd.pcd");
    std::cout << "点云匹配后融合：\n";
    CheckFuseResult(clouda_ptr_, cloudb_ptr_);
    FusePointCloud(clouda_ptr_, cloudb_ptr_, "icp_out_pcd.pcd");
}

// GICP匹配 获取更好的地图融合信息
void Cadd_pcd::GicpGetTransf()
{   
    pcl::PointCloud<pcl::PointXYZ> temp;
    temp = *clouda_ptr_;
    *cloudout_ptr_ = *clouda_ptr_ + *cloudb_ptr_;
    CheckPointCloud(cloudout_ptr_, "gps_out_pcd.pcd");
    std::cout << "GPS信息直接融合：\n";
    CheckFuseResult(clouda_ptr_, cloudb_ptr_, "gps");
    FusePointCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(temp), cloudb_ptr_, "gps_out_pcd.pcd");
    std::cout << "点云匹配ing\n匹配较为耗时 1min+\n";
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputTarget(clouda_ptr_);
    gicp.setInputSource(cloudb_ptr_);

    pcl::PointCloud<pcl::PointXYZ> output_ans;
    gicp.align(output_ans);
    cloudb_after_icp_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(output_ans);

    std::cout<< " score: " << gicp.getFitnessScore() << std::endl;
    
    icp_transf_ = gicp.getFinalTransformation();

    // *cloudout_ptr_ = *clouda_ptr_ + *cloudb_after_icp_ptr_;
    // CheckPointCloud(cloudout_ptr_, "icp_out_pcd.pcd");
    std::cout << "点云匹配后融合：\n";
    CheckFuseResult(clouda_ptr_, cloudb_after_icp_ptr_);
    FusePointCloud(clouda_ptr_, cloudb_after_icp_ptr_ , "icp_out_pcd.pcd");
}

// 融合结果度量
void Cadd_pcd::CheckFuseResult(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, std::string plot_name)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> target_cloud_kdtree;
    int search_num = 1;
    double sum_distance = 0, mean_distance = 0, variance_distance = 0, variance_temp_sum = 0;
    int distance_statistic[2001];  // 数组 索引 是两点云，最近点之间的距离，数组内容是最近点的距离落于该区间的点的个数
    int distance_statistic_index;
    std::vector<int> index_of_nearest(search_num);
    std::vector<float> squared_distance_of_nearest(search_num);
    std::vector<float> result_of_distance(input_cloud_ptr->size());
	std::vector<double> plot_x, plot_y;

    result_of_distance.clear(); 
    for (int i = 0; i <= 2000; i ++)
    {
        distance_statistic[i] = 0;
    }  

    target_cloud_kdtree.setInputCloud(target_cloud_ptr);
    #pragma omp for // OpenMP并行化执行这条语句后的for循环，从而起到加速的效果
    for (size_t i = 0; i < input_cloud_ptr->points.size(); i ++)
    {
        target_cloud_kdtree.nearestKSearch(input_cloud_ptr->points[i], search_num, index_of_nearest, squared_distance_of_nearest);
        sum_distance += sqrt(squared_distance_of_nearest[0]);
        result_of_distance.push_back(sqrt(squared_distance_of_nearest[0]));
        distance_statistic_index = int (sqrt(squared_distance_of_nearest[0]) * 100);  // 将两点之间的距离转为厘米，用于数组索引
        // 分类统计点的个数
        if (distance_statistic_index <= 2000)
        {
            distance_statistic[distance_statistic_index]++;
        }
        else
        {
            distance_statistic[2000]++;
        }
    }
    mean_distance = sum_distance / (input_cloud_ptr->size());
    std::for_each (std::begin(result_of_distance), std::end(result_of_distance), [&](const double distance){
                variance_temp_sum += (distance - mean_distance) * (distance - mean_distance);
    });
    variance_distance = variance_temp_sum / (input_cloud_ptr->size());
    std::cout << plot_name + ":\n" <<"均值：" << mean_distance << "\n方差 :" << variance_distance <<std::endl;

    //  绘图
    for (int i = 0; i <= 2000; i ++)
    {
        plot_x.push_back(i/100.0);
        plot_y.push_back(distance_statistic[i]);
    }
    matplotlibcpp::figure_size(1200, 780);
    matplotlibcpp::xlim(0, 21);
    matplotlibcpp::named_plot(plot_name + "figure", plot_x, plot_y);
    matplotlibcpp::xlabel("distance(m)");
    matplotlibcpp::ylabel("count of the point");
    matplotlibcpp::title(plot_name + " statistic ");
    matplotlibcpp::legend();
    matplotlibcpp::save("/home/cyppt/catkin_ws/src/try_pcd/" + plot_name + ".png");
    if (plot_name == "gps") 
    {
        plot_x_gps = plot_x;
        plot_y_gps = plot_y;
    }
    else if (plot_name == "icp")
    {
        plot_x_icp = plot_x;
        plot_y_icp = plot_y;
    }
} 

// 均值方差与图形化对比结果
void Cadd_pcd::ComparisonFuse()
{
    matplotlibcpp::figure_size(1200, 780);
    matplotlibcpp::xlim(0, 21);
    matplotlibcpp::named_plot("gps", plot_x_gps, plot_y_gps);
    matplotlibcpp::named_plot("icp", plot_x_icp, plot_y_icp);
    matplotlibcpp::xlabel("distance(m)");
    matplotlibcpp::ylabel("count of the point");
    matplotlibcpp::title("comparison statistic ");
    matplotlibcpp::legend();
    matplotlibcpp::save("/home/cyppt/catkin_ws/src/try_pcd/comparison.png");
    matplotlibcpp::show();
}

// 两幅点云融合
void Cadd_pcd::FusePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, std::string pcd_name)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> target_cloud_kdtree;
    int search_num = 1;
    double target_confidence_variable = 0.5;
    std::vector<int> index_of_nearest(search_num);
    std::vector<float> squared_distance_of_nearest(search_num);

    target_cloud_kdtree.setInputCloud(target_cloud_ptr);
    #pragma omp for // OpenMP并行化执行这条语句后的for循环，从而起到加速的效果
    for (size_t i = 0; i < input_cloud_ptr->points.size(); i ++)
    {
        target_cloud_kdtree.nearestKSearch(input_cloud_ptr->points[i], search_num, index_of_nearest, squared_distance_of_nearest);
        target_cloud_ptr->points[index_of_nearest[0]].x = (target_cloud_ptr->points[index_of_nearest[0]].x * target_confidence_variable + input_cloud_ptr->points[i].x * (1 - target_confidence_variable)) / 2.0;
        target_cloud_ptr->points[index_of_nearest[0]].y = (target_cloud_ptr->points[index_of_nearest[0]].y * target_confidence_variable + input_cloud_ptr->points[i].y * (1 - target_confidence_variable)) / 2.0;
        target_cloud_ptr->points[index_of_nearest[0]].z = (target_cloud_ptr->points[index_of_nearest[0]].z * target_confidence_variable + input_cloud_ptr->points[i].z * (1 - target_confidence_variable)) / 2.0;
    }
    CheckPointCloud(target_cloud_ptr, pcd_name);
}