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
#include "base_local_planner/trajectory_planner_ros.h"

// definition of poses
nav_msgs::Odometry pose_current;
geometry_msgs::PoseStamped pose_goal;
bool received_goal = false;

// callback for sub_current
void get_current_pose_callback(const nav_msgs::Odometry::ConstPtr& pose)
{
    pose_current = *pose;
    pose_current.header.frame_id = "map"; // change frame_id to "map" but not shure if it is right
}

// callback for sub_goal
void get_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    pose_goal = *pose;
    pose_goal.header.frame_id = "map"; // change frame_id to "map" but not shure if it is right
    received_goal = true;
}

// alternative callback for sub_goal
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

// onversion from type nav_msgs::Odometry to geometry_msgs::PoseStamped
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

// main
int main(int argc, char **argv)
{
    // initialization of node
    ros::init(argc, argv, "navi");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // setting up publishers and subscribers
    ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("key_vel", 1000);
    ros::Subscriber sub_current = n.subscribe("/mobile_base_controller/odom", 1000, get_current_pose_callback);
    ros::Subscriber sub_goal = n.subscribe("/move_base_simple/goal", 1000, get_goal_callback);

    // initialization fo vatriables needed for planning and executing route
    geometry_msgs::PoseStamped pose_start;
    std::vector<geometry_msgs::PoseStamped> plan;
    geometry_msgs::Twist vel;
    bool reached_goal = true;
    bool is_going= false;

    // sth here didn't want to work, and I had to paste this line to command line:
    // sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
    // setting up costmaps
    tf2_ros::Buffer buf(ros::Duration(10));
    tf2_ros::TransformListener tf(buf); // idk if this is needed
    costmap_2d::Costmap2DROS costmap_local("costmap_local", buf);
    costmap_2d::Costmap2DROS costmap_global("costmap_global", buf);

    // rcovery behavior for when local planner gets stuck
    rotate_recovery::RotateRecovery recovery;


    // initialization of local planer
    // dwa_local_planner::DWAPlannerROS planner_local;
    // planner_local.initialize("planner_local", &buf, &costmap_local);

    base_local_planner::TrajectoryPlannerROS planner_local;
    planner_local.initialize("planner_local", &buf, &costmap_local);

    // initialization of global planer, idk which is better is better/the right one

    global_planner::GlobalPlanner planner_global;
    planner_global.initialize("planner_global", &costmap_global);

    // global_planner::GlobalPlanner planner_global("planner_global", costmap_global.getCostmap(), "map");

    int count = 0;
    while (ros::ok())
    {
        if (received_goal) {
            // if new one is given, in this part the plan will be generated
            pose_start = odom2pose(pose_current); // convert pose_current which is Odometry to PoseStamped
            pose_start.header.frame_id = "map"; // change frame_id to "map" but not shure if it is right
            bool make_plan_status = planner_global.makePlan(pose_start, pose_goal, plan); // global planner makes plan
            bool set_plan_status = planner_local.setPlan(plan); // plan from global planner is passed to the local planner
            if (make_plan_status && set_plan_status) {
                // when everything was calculated correctly moves on to moving
                is_going = true;
                received_goal = false;
            } else ROS_INFO("Problem with making and setting plan");
        }
        if (!is_going) {
            // happens when goal is reached, so there is no new goal
            
        } else {
            // check if local planner found a trajectory
            bool found_trajectory = planner_local.computeVelocityCommands(vel); // could use dwaComputeVelocityCommands(), but needs 2 arguments
            if (found_trajectory) {
                // if it did publish instructions
                pub_vel.publish(vel);
                ROS_INFO("Proceeding to target");
            } else {
                // if it didn't run recovery
                recovery.initialize("recovery", &buf, &costmap_global, &costmap_local);
                recovery.runBehavior();

                ROS_INFO("Recovery behavior");
            }
            reached_goal = planner_local.isGoalReached(); // check if local planner has reached a goal
            if (reached_goal) is_going = false;
        }
        // ROS_INFO("%s", msg.data.c_str());

        // basics needed for publishing/subscribing and while loop tooperate smoothly
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
