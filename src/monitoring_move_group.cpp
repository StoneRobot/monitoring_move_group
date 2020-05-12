#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>

void addData(double posi[], const geometry_msgs::PoseStamped& pose)
{
    posi[0] = pose.pose.position.x;
    posi[1] = pose.pose.position.y;
    posi[2] = pose.pose.position.z;
}

double distance(double pose1[], double pose2[])
{
    double x = pose1[0] - pose2[0];
    double y = pose1[1] - pose2[1];
    double z = pose1[2] - pose2[2];
    return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "monitoring_move_group");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface move_group0("arm0");
    moveit::planning_interface::MoveGroupInterface move_group1("arm1");
    geometry_msgs::PoseStamped pose0;
    geometry_msgs::PoseStamped pose1;
    double position0[3];
    double position1[3];
    double data;
    while (ros::ok())
    {
        pose0 = move_group0.getCurrentPose();
        pose1 = move_group1.getCurrentPose();
        addData(position0, pose0);
        addData(position1, pose1);
        data = distance(position0, position1);
        if(data < 0.01)
        {
        }
    }
    return 0;
}