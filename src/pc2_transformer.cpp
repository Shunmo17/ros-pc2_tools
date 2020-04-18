#include <string>
#include <time.h>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "pcl_ros/transforms.h"

class Pc2Transformer
{
private:
  ros::NodeHandle nh;

public:
  Pc2Transformer(std::string, std::string, std::string);
  void pcd_callback(const sensor_msgs::PointCloud2::ConstPtr &);
  sensor_msgs::PointCloud2 transform_pcd(const sensor_msgs::PointCloud2::ConstPtr &);

  ros::Subscriber sub;
  ros::Publisher pub;
  tf::StampedTransform transform;
  tf::TransformListener listener;
  std::string new_frame_id;
};

Pc2Transformer::Pc2Transformer(std::string _input_pc2_topic, std::string _output_pc2_topic, std::string _new_frame_id)
{
  new_frame_id = _new_frame_id;
  sub = nh.subscribe(_input_pc2_topic, 1, &Pc2Transformer::pcd_callback, this);
  pub = nh.advertise<sensor_msgs::PointCloud2>(_output_pc2_topic, 1);
}

void Pc2Transformer::pcd_callback(const sensor_msgs::PointCloud2::ConstPtr &_original_pcd)
{
  // transform original frame to base_link
  sensor_msgs::PointCloud2 transformed_pcd;
  transformed_pcd = this->transform_pcd(_original_pcd);

  // publish transformed pcd
  pub.publish(transformed_pcd);
}

sensor_msgs::PointCloud2 Pc2Transformer::transform_pcd(const sensor_msgs::PointCloud2::ConstPtr &_original_pcd)
{
  std::string frame_id;
  sensor_msgs::PointCloud2 transformed_pcd;
  frame_id = _original_pcd->header.frame_id;
  while (ros::ok())
  {
    try
    {
      listener.lookupTransform(new_frame_id, frame_id, ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(0.001).sleep();
    }
  }
  pcl_ros::transformPointCloud(new_frame_id, transform, *_original_pcd, transformed_pcd);
  return transformed_pcd;
}

//===========================================================================================================//
//                                               Main Function                                               //
//===========================================================================================================//
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcd_transformer");

  ros::NodeHandle pnh("~");

  // get parameters
  std::string input_pc2_topic, output_pc2_topic, new_frame_id;
  pnh.getParam("input_pc2_topic", input_pc2_topic);
  pnh.getParam("output_pc2_topic", output_pc2_topic);
  pnh.getParam("new_frame_id", new_frame_id);

  Pc2Transformer pc2_transformer(input_pc2_topic, output_pc2_topic, new_frame_id);

  ros::spin();
}