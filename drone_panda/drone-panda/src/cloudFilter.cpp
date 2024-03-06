//////////////////////////////////////////////////////////////////////////
// PCLPlanes.cpp               
// This file is responsible for subscribing to the final mapped topics
// and generating a list of relevant planes 
//////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
// PCL-specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Export to custom message type
#include "drone_ros_msgs/PlanesInliers.h"
#include "drone_ros_msgs/PlanesInliersArr.h"

// Segmentation-specific includes
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher cloudpub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for PCL2, PCL and temp point clouds for filtering data
    pcl::PCLPointCloud2* cloud2 = new pcl::PCLPointCloud2;     
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Convert from msg to PCL2 to PCL
    pcl_conversions::toPCL(*cloud_msg, *cloud2);
    pcl::fromPCLPointCloud2(*cloud2, *cloud);
    std::cout << "Original Cloud Size: " << cloud->size() << std::endl;

    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(-1.2, -1.2, -0.1, 1.0));
    boxFilter.setMax(Eigen::Vector4f(1.2, 1.4, 0.6, 1.0));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*cloud);
    pcl::toPCLPointCloud2(*cloud,*cloud2);
    sensor_msgs::PointCloud2* cloud_msg2 = new sensor_msgs::PointCloud2;
    pcl_conversions::fromPCL(*cloud2, *cloud_msg2);
    std::cout << "Filtered Cloud Size: " << cloud->size() << std::endl;
    cloudpub.publish(*cloud_msg2);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_plane_detect");
  ros::NodeHandle nh;

//   Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/filtered_cloud", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  cloudpub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_cloud2", 1, true);

  // Spin
  ros::spin ();
}