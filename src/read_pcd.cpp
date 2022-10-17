#include "ros/ros.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>

static std::string PATH = "/home/cyppt/Downloads/LOAM_HX18_320s/GlobalMap.pcd";

class Cread_pcd
{
private:
    ros::NodeHandle nh_;
    ros::Publisher pcl_pub_;
    pcl::PointCloud<pcl::PointXYZ> cloud_;
    sensor_msgs::PointCloud2 output_;
public:
    Cread_pcd();
    void Pub_pcd();
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "read_pcd");
    Cread_pcd read_pcd1;

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        read_pcd1.Pub_pcd();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

Cread_pcd::Cread_pcd()
{
    pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_output",  1);
    pcl::io::loadPCDFile(PATH, cloud_);
    pcl::toROSMsg(cloud_,output_);
    output_.header.frame_id = "map";  // fixed_fram ----rviz
}

void Cread_pcd::Pub_pcd()
{
    pcl_pub_.publish(output_); 
}