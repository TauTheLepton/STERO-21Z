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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navi");
    ros::NodeHandle n;
    ros::Publisher pub_vel = n.advertise<geometry_msgs::Twist>("topic", 1000); // set the right topic to publish velocity
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
