// Progam to extract sidewalk from pointcloud

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/Vertices.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>

#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <boost/thread/thread.hpp>

ros::Publisher pub;
ros::Publisher pub_out;

float deg2rad(float alpha)
{
  return (alpha * 0.017453293f);
}

void ransac(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  *cloud = *input;

  pcl::ModelCoefficients::Ptr floor_coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr floor_indices(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZRGB> floor_finder;
  floor_finder.setOptimizeCoefficients(true);
  floor_finder.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  //floor_finder.setModelType (SACMODEL_PLANE);
  floor_finder.setMethodType(pcl::SAC_RANSAC);
  floor_finder.setMaxIterations(50);
  floor_finder.setAxis(Eigen::Vector3f(1, 0, 0));
  floor_finder.setDistanceThreshold(1);
  floor_finder.setEpsAngle(deg2rad(20));
  floor_finder.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(*cloud));
  floor_finder.segment(*floor_indices, *floor_coefficients);

  if (floor_indices->indices.size() > 0)
  {
    // Extract the floor plane inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr floor_points(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extractor;
    extractor.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(*cloud));
    extractor.setIndices(floor_indices);
    extractor.filter(*floor_points);
    //extractor.setNegative(true);
    extractor.filter(*cloud);

    // Project the floor inliers
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(floor_points);
    proj.setModelCoefficients(floor_coefficients);
    proj.filter(*cloud_projected);

    floor_points->header.frame_id = "camera_depth_optical_frame";
    floor_points->header.stamp = ros::Time::now().toNSec();
   }
}


void passthrough_y(const sensor_msgs::PointCloud2ConstPtr& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(*cloud));
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-20.0, 0.5);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_passthrough);
}

void passthrough_oulier(const sensor_msgs::PointCloud2ConstPtr& input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(*cloud));
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-20.0, 0.5);
    pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_passthrough);
}



void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Do data processing here...
  
  // run pass through filter to shrink point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZRGB>);
  //passthrough_z(input, cloud_passthrough);
  passthrough_y(input,cloud_passthrough);
  //passthrough_x(cloud_passthrough);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outlier(new pcl::PointCloud<pcl::PointXYZRGB>);
  passthrough_oulier(input,cloud_outlier);
  pub_out.publish(*cloud_outlier);

  // run ransac to find floor
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZRGB>);
  ransac(cloud_passthrough, cloud_projected);
  pub.publish(*cloud_projected);

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_realsense_pkg");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/camera/depth/points_in", 1);

  pub_out = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth/points_out", 1);

  // Spin
  ros::spin ();
}
