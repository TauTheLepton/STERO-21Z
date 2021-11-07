#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf2_ros/transform_listener.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "dwa_local_planner/dwa_planner_ros.h"
#include "global_planner/planner_core.h"
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navi");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Rate loop_rate(10);

    // sth here didn't want to work, and I had to paste this line to command line:
    // sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
    tf2_ros::Buffer buf(ros::Duration(10));
    tf2_ros::TransformListener tf(buf);
    costmap_2d::Costmap2DROS costmap_local("costmap_local", buf);
    costmap_2d::Costmap2DROS costmap_global("costmap_global", buf);

    dwa_local_planner::DWAPlannerROS dp;
    dp.initialize("my_dwa_planner", &buf, &costmap_local);

    global_planner::GlobalPlanner planner_global("planner_global", costmap_global.getCostmap(), "map");

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
