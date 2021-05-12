//include statements
#include <ros/ros.h>
#include <mobot_pub_des_state/path.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <queue>
#include <traj_builder/traj_builder.h> //has almost all the headers we need
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <mobot_pub_des_state/path.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <exception>

#include <odom_tf/odom_tf.h>
#include <xform_utils/xform_utils.h>

//pcl includes:
#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //use these to convert between PCL and ROS datatypes
//#include <pcl/ros/conversions.h>
#include <pcl/conversions.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/PCLHeader.h>

#include <pcl_utils/pcl_utils.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 

#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLHeader.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 

#include <abc_final/ObjectGrabberService.h>

#include <baxter_core_msgs/AssemblyState.h>

ros::ServiceClient grabber_client;
ros::ServiceClient dropper_client;

geometry_msgs::Pose block_array[9];
geometry_msgs::Pose jenga_array[9];

void initializeServiceClients(ros::NodeHandle n){

  grabber_client = n.serviceClient<abc_final::ObjectGrabberService> ("grabber_service",1);
  
  while (!grabber_client.exists()) {
    ROS_INFO("waiting for grabber service...");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("connected client to service");
  
  dropper_client = n.serviceClient<abc_final::ObjectGrabberService> ("dropper_service",1);
  
  while (!dropper_client.exists()) {
    ROS_INFO("waiting for dropper service...");
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("connected client to service");
}

geometry_msgs::Pose quick_pose(double x, double y, double z, double a, double b, double c, double w){
  geometry_msgs::Pose dummy_pose;

  dummy_pose.position.x = x;
  dummy_pose.position.y = y;
  dummy_pose.position.z = z;

  dummy_pose.orientation.x = a;
  dummy_pose.orientation.y = b;
  dummy_pose.orientation.z = c;
  dummy_pose.orientation.w = w;

  return dummy_pose;
}

void set_block_poses(){
  geometry_msgs::Pose block1 = quick_pose(0.48, 0.14, 0.775, 0, 0, -0.6568247, 0.7540433);
  geometry_msgs::Pose block2 = quick_pose(0.718, -0.2, 0.775, 0, 0, -0.306999, 0.9517098);
  geometry_msgs::Pose block3 = quick_pose(0.75, -0.095, 0.775, 0, 0, -0.0625373, 0.9980426);
  geometry_msgs::Pose block4 = quick_pose(0.625, 0.06, 0.775, 0, 0, -0.6378512, 0.7701596);
  geometry_msgs::Pose block5 = quick_pose(0.56, -0.3, 0.775, 0, 0, 0.6329812, 0.7741672);
  geometry_msgs::Pose block6 = quick_pose(0.74, 0.05, 0.775, 0, 0, 0.4527955, 0.8916144);
  geometry_msgs::Pose block7 = quick_pose(0.69, -0.29, 0.775, 0, 0, -0.3054378, 0.952212);
  geometry_msgs::Pose block8 = quick_pose(0.5, -0.08, 0.775, 0, 0, 0.707, 0.707);
  geometry_msgs::Pose block9 = quick_pose(0.465, -0.29, 0.775, 0, 0, -0.5745246, 0.8184873);

  block_array[0] = block8; 
  block_array[1] = block1; 
  block_array[2] = block4; 
  block_array[3] = block9; 
  block_array[4] = block2; 
  block_array[5] = block6; 
  block_array[6] = block7; 
  block_array[7] = block3; 
  block_array[8] = block5;
}

void set_jenga_poses(){
  geometry_msgs::Pose jenga1 = quick_pose(0.5, -0.0825, 0.8, 0, 0, 0, 1);
  geometry_msgs::Pose jenga2 = quick_pose(0.5, 0, 0.8, 0, 0, 0, 1);
  geometry_msgs::Pose jenga3 = quick_pose(0.5, 0.0825, 0.8, 0, 0, 0, 1);
  geometry_msgs::Pose jenga4 = quick_pose(0.4175, 0, 0.85, 0, 0, 0.707, 0.707);
  geometry_msgs::Pose jenga5 = quick_pose(0.5, 0, 0.85, 0, 0, 0.707, 0.707);
  geometry_msgs::Pose jenga6 = quick_pose(0.5825, 0, 0.85, 0, 0, 0.707, 0.707);
  geometry_msgs::Pose jenga7 = quick_pose(0.5, -0.0825, 0.9, 0, 0, 0, 1);
  geometry_msgs::Pose jenga8 = quick_pose(0.5, 0, 0.9, 0, 0, 0, 1);
  geometry_msgs::Pose jenga9 = quick_pose(0.5, 0.0825, 0.9, 0, 0, 0, 1);

  jenga_array[0] = jenga2; 
  jenga_array[1] = jenga1; 
  jenga_array[2] = jenga3; 
  jenga_array[3] = jenga4; 
  jenga_array[4] = jenga5; 
  jenga_array[5] = jenga6; 
  jenga_array[6] = jenga7; 
  jenga_array[7] = jenga8; 
  jenga_array[8] = jenga9;
}

//main function
int main(int argc, char **argv) {
	//ros init
  ros::init(argc, argv, "abc_jenga");
  ros::NodeHandle n;
  
  initializeServiceClients(n);

  abc_final::ObjectGrabberService grabber_srv;
  abc_final::ObjectGrabberService dropper_srv;

  ROS_INFO("Configuring array of block poses");
  set_block_poses();

  ROS_INFO("Configuring array of Jenga poses");
  set_jenga_poses();


  for(int i=0;i<9;i++){
    ROS_INFO("Getting pose for block %i",i);
    grabber_srv.request.des_pose = block_array[i];

    ROS_INFO("Calling grabber service for block %i",i);
    grabber_client.call(grabber_srv);

    ROS_INFO("Block grabbed, ready to move");

    ros::Duration(1).sleep();

    dropper_srv.request.des_pose = jenga_array[i];

    ROS_INFO("Calling dropper service for block %i",i);
    dropper_client.call(dropper_srv);

    ROS_INFO("Finished dropping, moving on");
    ros::Duration(1).sleep();

  }

  ROS_INFO("Finished moving all blocks");

  return 0;
}
