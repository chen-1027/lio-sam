#include "../include/utility.h"
#include "lio_sam/cloud_info.h"
#include <pcl/filters/passthrough.h>
// struct VelodynePointXYZIRT
// {
//     PCL_ADD_POINT4D
//     PCL_ADD_INTENSITY;
//     uint16_t ring;
//     float time;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;
// POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
//     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
//     (uint16_t, ring, ring) (float, time, time)
// )
// using PointXYZIRT = VelodynePointXYZIRT;

class pointProjection :public ParamServer
{
private:
    ros::Subscriber subLaserCloud;
    ros::Publisher pubLaserCloud;


public:
    pointProjection(){
    subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("rslidar_points", 5, &pointProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("rs_point",5);
    };
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
        //pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
        pcl::PointCloud<pcl::PointXYZI>::Ptr points(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr points_filter(new pcl::PointCloud<pcl::PointXYZI>);
         pcl::fromROSMsg(*laserCloudMsg,*points);
        //pcl::moveFromROSMsg(*laserCloudMsg, *laserCloudIn);
        //地面上方剔除
        Eigen::Affine3f transFinal = pcl::getTransformation(0, 0, 0, 0, 0.479, 0);
        pcl::transformPointCloud(*points,*points_filter,transFinal);
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(points_filter);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-100,-0.54);
        pass.setFilterLimitsNegative(false);
        pass.filter(*points);

        sensor_msgs::PointCloud2 msg_filter;
        pcl::toROSMsg(*points,msg_filter);
        //pcl::toROSMsg(*reflector_points_i,msg_reflector);

        msg_filter.header.frame_id = "velodyne";
        msg_filter.header.stamp = laserCloudMsg->header.stamp;
        pubLaserCloud.publish(msg_filter);
    };
    ~pointProjection(){};
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    pointProjection PP;
    
    ROS_INFO("\033[1;32m----> Point Projection Started.\033[0m");
    //多线程处理
    ros::spin();
    
    return 0;
}


