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

ros::Publisher enable_pub;

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


//main function
int main(int argc, char **argv) {
	//ros init
  ros::init(argc, argv, "abc_jenga");
  ros::NodeHandle n;
  
  initializeServiceClients(n);

  abc_final::ObjectGrabberService grabber_srv;
  abc_final::ObjectGrabberService dropper_srv;

  ROS_INFO("Supplying discrete centroid");

  geometry_msgs::Pose dummy_pose;

  grabber_srv.request.des_pose = quick_pose(0.725, -0.1, 0.775, 0.0, 0.0, -0.8230226, -0.5680087);
 
  ROS_INFO("Pose declared at (%f,%f,%f)",grabber_srv.request.des_pose.position.x,grabber_srv.request.des_pose.position.y,grabber_srv.request.des_pose.position.z);

  ROS_INFO("Calling grabber service for block 1");
  
  grabber_client.call(grabber_srv);
  
  ROS_INFO("Block grabbed, ready to move");

  ros::Duration(3).sleep();

  dropper_srv.request.des_pose = quick_pose(0.55, -0.3, 0.9, 0.0, 0.0, -0.3826834, 0.9238795);

  ROS_INFO("Block ready to drop");
  
  dropper_client.call(dropper_srv);
  
  ROS_INFO("Finished dropping");

  return 0;
}
