#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf2_ros/transform_listener.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "dwa_local_planner/dwa_planner_ros.h"
#include "global_planner/planner_core.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include "rotate_recovery/rotate_recovery.h"
#include "nav_msgs/Odometry.h"

nav_msgs::Odometry pose_current;
geometry_msgs::PoseStamped pose_goal;

void get_current_pose_callback(const nav_msgs::Odometry::ConstPtr& pose)
{
    pose_current = *pose;
    pose_current.header.frame_id = "map";
}

void get_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    pose_goal = *pose;
    pose_goal.header.frame_id = "map";
}

// void goal_callback(const geometry_msgs::PoseStamped::SharedPtr msg){
//     // ROS_INFO_STREAM("Received pose: " << msg);
//     // msg->pose.pose.position.x
//     // msg->pose.pose.position.y
//     // msg->pose.pose.position.theta

//     //position in map frame
//     double tx = msg->pose.position.x;
//     double ty = msg->pose.position.y;

//     //orientation quaternion
//     tf2::Quaternion q(
//                 msg->pose.orientation.x,
//                 msg->pose.orientation.y,
//                 msg->pose.orientation.z,
//                 msg->pose.orientation.w);

//     // 3x3 Rotation matrix from quaternion
//     tf2::Matrix3x3 m(q);

//     // Roll Pitch and Yaw from rotation matrix
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw);

//     // Output the measure
//     RCLCPP_INFO(get_logger(), "Received pose in '%s' frame : X: %.2f Y: %.2f - R: %.2f P: %.2f Y: %.2f - Timestamp: %u.%u sec ",
//                 msg->header.frame_id.c_str(),
//                 tx, ty,
//                 roll, pitch, yaw ,
//                 msg->header.stamp.sec,msg->header.stamp.nanosec);
// }

// void g_costmap_callback(const geometry_msgs::PoseStamped::SharedPtr msg){

// }

geometry_msgs::PoseStamped odom2pose(nav_msgs::Odometry odom)
{
    geometry_msgs::PoseStamped my_pose;
    my_pose.pose.position.x = odom.pose.pose.position.x;
    my_pose.pose.position.y = odom.pose.pose.position.y;
    my_pose.pose.position.z = odom.pose.pose.position.z;
    my_pose.pose.orientation.w = odom.pose.pose.orientation.w;
    my_pose.pose.orientation.x = odom.pose.pose.orientation.x;
    my_pose.pose.orientation.y = odom.pose.pose.orientation.y;
    my_pose.pose.orientation.z = odom.pose.pose.orientation.z;
    my_pose.header.frame_id = odom.header.frame_id;
    my_pose.header.seq = odom.header.seq;
    my_pose.header.stamp = odom.header.stamp;
    return my_pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navi");
    ros::NodeHandle n;

    ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("key_vel", 1000);
    ros::Subscriber sub_current = n.subscribe("/mobile_base_controller/odom", 1000, get_current_pose_callback);
    ros::Subscriber sub_goal = n.subscribe("/move_base_simple/goal", 1000, get_goal_callback);
    ros::Rate loop_rate(10);

    geometry_msgs::PoseStamped pose_start;
    std::vector<geometry_msgs::PoseStamped> plan;

    geometry_msgs::Twist vel;

    bool reached_goal = true;

    // sth here didn't want to work, and I had to paste this line to command line:
    // sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
    tf2_ros::Buffer buf(ros::Duration(10));
    tf2_ros::TransformListener tf(buf); // idk if this is needed
    costmap_2d::Costmap2DROS costmap_local("costmap_local", buf);
    costmap_2d::Costmap2DROS costmap_global("costmap_global", buf);

    rotate_recovery::RotateRecovery recovery;
    recovery.initialize("recovery", &buf, &costmap_global, &costmap_local);

    dwa_local_planner::DWAPlannerROS planner_local;
    planner_local.initialize("planner_local", &buf, &costmap_local);

    // idk which initialization for GlobalPlanner is better, but as for now neither work

    // global_planner::GlobalPlanner planner_global;
    // planner_global.initialize("planner_global", &costmap_global);

    global_planner::GlobalPlanner planner_global("planner_global", costmap_global.getCostmap(), "map");

    int count = 0;
    while (ros::ok())
    {
        if (reached_goal) {
            // pose_start = pose_current;
            pose_start.header.frame_id = "map";
            pose_start = odom2pose(pose_current);
            planner_global.makePlan(pose_start, pose_goal, plan);
            planner_local.setPlan(plan);
        } else {
            reached_goal = planner_local.isGoalReached();
            bool found_trajectory = planner_local.computeVelocityCommands(vel); // could use dwaComputeVelocityCommands(), but needs 2 arguments
            if (found_trajectory) {
                pub_vel.publish(vel);
            } else {
                recovery.runBehavior();
            }
        }
        // ROS_INFO("%s", msg.data.c_str());

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
