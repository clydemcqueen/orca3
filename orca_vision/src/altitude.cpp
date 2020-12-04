#include <iostream>
#include "pcl/ModelCoefficients.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/Marker.h"
#include "pcl_ros/point_cloud.h"
#include "tf2_eigen/tf2_eigen.h"

ros::Publisher g_marker_pub;

// New stereo image
// TODO can I do this w/ a disparity image instead?
void points2Callback(const pcl::PCLPointCloud2::ConstPtr& msg)
{
  // TODO
  // subscribe to the stereo image
  // run through this algo
  // if we find a plane, calc the distance to it
  // display a marker in rviz
  // run a simulation in gazebo

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromPCLPointCloud2(*msg, *cloud);

#if 0
  // Fill in the cloud data
  cloud->width  = 15;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  // Generate the data
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1.0;
  }

  // Set a few outliers
  cloud->points[0].z = 2.0;
  cloud->points[3].z = -2.0;
  cloud->points[6].z = 4.0;

  std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " "
      << cloud->points[i].y << " "
      << cloud->points[i].z << std::endl;
#endif

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.05);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return;
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
    << coefficients->values[1] << " "
    << coefficients->values[2] << " "
    << coefficients->values[3] << std::endl;

  Eigen::Vector3d v1 = {coefficients->values[0], coefficients->values[1], coefficients->values[2]};
  Eigen::Vector3d v2 = {0, 0, 1};

  Eigen::Quaterniond q;
  q.setFromTwoVectors(v2, v1);

  visualization_msgs::Marker ground_plane;
  ground_plane.header.frame_id = "left_camera_frame";
  ground_plane.header.stamp = ros::Time();
  ground_plane.ns = "ground";
  ground_plane.id = 0;
  ground_plane.type = visualization_msgs::Marker::CUBE;
  ground_plane.action = visualization_msgs::Marker::ADD;
  ground_plane.pose.position.x = 0;
  ground_plane.pose.position.y = 0;
  ground_plane.pose.position.z = 1; // TODO
  ground_plane.pose.orientation = tf2::toMsg(q);
  ground_plane.scale.x = 1;
  ground_plane.scale.y = 1;
  ground_plane.scale.z = 0.1;
  ground_plane.color.a = 0.2;
  ground_plane.color.r = 1.0;
  ground_plane.color.g = 1.0;
  ground_plane.color.b = 1.0;
  g_marker_pub.publish(ground_plane);

  // Publish the inliers as a point cloud?

#if 0
  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
      << cloud->points[inliers->indices[i]].y << " "
      << cloud->points[inliers->indices[i]].z << std::endl;
#endif
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "altitude");
  ros::NodeHandle nh{""};
  ros::NodeHandle nh_priv{"~"};

  ros::Subscriber point_cloud_sub = nh_priv.subscribe<pcl::PCLPointCloud2>("/stereo/points2", 10, &points2Callback);
  g_marker_pub = nh_priv.advertise<visualization_msgs::Marker>("/ground_plane_marker", 1);

  // TODO publish altitude as well

  ROS_INFO("Entering main loop");
  ros::Rate r(50);
  while (ros::ok())
  {
    // Respond to incoming messages
    ros::spinOnce();

    // Wait
    r.sleep();
  }

  return 0;
}