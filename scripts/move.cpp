#include "ros/ros.h"
// #include "geometry_msgs/PoseStamped.h"
// #include "geometry_msgs/PoseWithCovariance.h"
// #include "tf2_ros/transform_listener.h"
// #include "costmap_2d/costmap_2d_ros.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"
// #include "global_planner/planner_core.h"

geometry_msgs::PoseStamped goalPose;
geometry_msgs::PoseStamped currentPose;
std::vector<geometry_msgs::PoseStamped> globalPlannerVector;
bool activateGlobalPlanner = false;

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  goalPose.pose = msg->pose;
  activateGlobalPlanner = true;
}

void actualPoseCallback(const geometry_msgs::PoseWithCovariance::ConstPtr& msg)
{
  currentPose.pose = msg->pose;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "custom_move_base");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/move_base_simple/goal", 1000, goalCallback);
  ros::Subscriber subOdom = n.subscribe("/mobile_base_controller/odom/pose", 1000, actualPoseCallback);
  ros::Rate loop_rate(10);

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(buffer);
  //craeting global costmap
  costmap_2d::Costmap2DROS globalCostmap("global_costmap", buffer);
  //creating local costmap
  costmap_2d::Costmap2DROS localCostmap("local_costmap", buffer);
  global_planner::GlobalPlanner globalPlanner("global_planner", globalCostmap.getCostmap(), "map");
  base_local_planner::TrajectoryPlannerROS tp;
  tp.initialize("my_trajectory_planner", &tf, &localCostmap)
  while (ros::ok())
  {
    if (activateGlobalPlanner)
    {
      globalPlanner.makePlan(currentPose, goalPose, globalPlannerVector);
      activateGlobalPlanner = false;  
    }
    printf("Current x: %2.2f", currentPose.pose.position.x);
    printf("Goal x: %2.2f", goalPose.pose.position.x);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}