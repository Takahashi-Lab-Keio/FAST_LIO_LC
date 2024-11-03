
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <math.h> 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
// PCL includes
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl_ros/transforms.h>

#include <Eigen/Geometry>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "math.h"
class PointcloudColorizer
{
    ros::NodeHandle nh_;
    tf::TransformBroadcaster br;
    image_transport::ImageTransport it_;
    ros::Subscriber lidar_pcd_sub;
    ros::Subscriber cam_info_sub;
    image_transport::Subscriber cam_image_sub;
    ros::Publisher lid_color_pcd_pub;
    
    std::string lidar_pcd_topic_ = "/cloud_registered_body";

    tf::Transform lidar_camera_transform;
    tf::Transform body_imu_transform;

    sensor_msgs::ImageConstPtr cam_msg_ptr;
    bool cam_is_subscribed = false; 
    sensor_msgs::PointCloud2ConstPtr pc2_ptr;
    pcl::PointCloud<pcl::PointXYZRGB> colorized_cloud;
    bool range_is_subscribed = false; 

    sensor_msgs::CameraInfo cam_info_msg;
    image_geometry::PinholeCameraModel model;
    bool cam_info_msg_is_subscribed = false; 

public:
    PointcloudColorizer(): it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        lidar_pcd_sub = nh_.subscribe(lidar_pcd_topic_, 1, &PointcloudColorizer::lidarPcdCb, this);
        lid_color_pcd_pub = nh_.advertise<sensor_msgs::PointCloud2>("/color_cloud_registered_body", 1);
      
        ros::spin();
    }
    ~PointcloudColorizer()
    {
    }
    void lidarPcdCb(const sensor_msgs::PointCloud2ConstPtr& msg)
    {   
        sensor_msgs::PointCloud2 pc2_ros = this->ColorizePointCloud(msg);
        pc2_ros.header = msg->header;
        lid_color_pcd_pub.publish( pc2_ros);
    }

    sensor_msgs::PointCloud2 ColorizePointCloud(sensor_msgs::PointCloud2ConstPtr lid_msg){
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZINormal>());
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_transed(new pcl::PointCloud<pcl::PointXYZINormal>());
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZINormal>());
        pcl::fromROSMsg (*lid_msg, *cloud_in);
        pcl_ros::transformPointCloud(*cloud_in, *cloud_transed,lidar_camera_transform);
        cloud_out->points.resize(cloud_in->points.size());
        for(int i = 0 ; i < cloud_in->points.size() ; i++)
        {   
            cloud_out->points[i] = cloud_in->points[i];
            // cloud_out->points[i].x = cloud_in->points[i].x;
            // cloud_out->points[i].y = cloud_in->points[i].y;
            // cloud_out->points[i].z = cloud_in->points[i].z;
            // cloud_out->points[i].a = cloud_in->points[i].intensity;
            // cloud_out->points[i].r = 100;
            // cloud_out->points[i].g = 0;
            // cloud_out->points[i].b = 0;
            // std::cout << cloud_in->points[i].normal_x << " " << cloud_in->points[i].normal_y  << std::endl;
        }
        sensor_msgs::PointCloud2 pc2_ros;
        pcl::PCLPointCloud2 pc2_pcl;
        pcl::toPCLPointCloud2(*cloud_out,pc2_pcl);
        pcl_conversions::moveFromPCL(pc2_pcl, pc2_ros);
        
        return pc2_ros;
    }
    sensor_msgs::PointCloud2 PclToRos(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_pcl){
        sensor_msgs::PointCloud2 pc2_ros;
        pcl::PCLPointCloud2 pc2_pcl;
        pcl::toPCLPointCloud2(*pcd_pcl,pc2_pcl);
        pcl_conversions::moveFromPCL(pc2_pcl, pc2_ros);
        return pc2_ros;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "PointcloudColorizer");
    PointcloudColorizer cd;
    
    return 0;
}