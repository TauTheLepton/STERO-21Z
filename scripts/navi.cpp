#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf2_ros/transform_listener.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "dwa_local_planner/dwa_planner_ros.h"
#include "global_planner/planner_core.h"
#include <sstream>
#include "geometry_msgs/PoseStamped.h"
#include "rotate_recovery/rotate_recovery.h"

void my_callback(const std_msgs::StringConstPtr& str)
{
    double i=1;
    printf("Current i = %2.2f", i);
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navi");
    ros::NodeHandle n;
    // topic is '/key_vel' i guess. or '/nav_vel'???
    // position is maybe '/ground_truth_odom'?
    // '/input_joy/cmd_vel' another vel???
    // Subscriber 
    //ros::Subscriber goal_sub = n.subscribe("move_base_simple/goal", 1000, goal_callback);
    //ros::Subscriber g_costmap_sub = n.subscribe("map", 1000, g_costmap_callback);
    ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("key_vel", 1000); // set the right topic to publish velocity
    // ros::Subscriber sub = n.subscribe("some_topic", 1000, my_callback);
    ros::Rate loop_rate(10);

    std::vector<geometry_msgs::PoseStamped> plan;
    geometry_msgs::PoseStamped pose_start;
    geometry_msgs::PoseStamped pose_goal;
    geometry_msgs::Twist vel;

    bool reached_goal = false;

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

    global_planner::GlobalPlanner planner_global;
    planner_global.initialize("planner_global", &costmap_global);

    planner_global.makePlan(pose_start, pose_goal, plan);
    planner_local.setPlan(plan);

    int count = 0;
    while (ros::ok())
    {
        if (!reached_goal) {
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
