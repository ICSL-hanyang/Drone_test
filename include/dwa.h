#ifndef DWA_H
#define DWA_H

#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/TwistStamped.h>

typedef struct dynamic_window
{
	float vx_min_;
	float vx_max_;
	float vy_min_;
	float vy_max_;
	float vz_min_;
	float vz_max_;
} DynamicWindow;

class DWA
{
private:
    std::string &vehicle_name_;
    ros::NodeHandle &nh_;
    ros::NodeHandle &nh_global_;
    ros::Subscriber vel_sub_;
    double dt_;
    tf2::Vector3 cur_vel_;

    void velCB(const geometry_msgs::TwistStamped::ConstPtr &);
    DynamicWindow calDynamicWindow();
    std::vector<tf2::Vector3> calTrjectory(const tf2::Vector3 &, const tf2::Vector3 &);
    double calToGoalCost(std::vector<tf2::Vector3> &, const tf2::Vector3 &);
    double calObstacleCost(const tf2::Vector3 &, std::vector<tf2::Vector3> &, const std::vector<tf2::Vector3> &);
public:
    tf2::Vector3 evalTrajectory(const tf2::Vector3 &, const tf2::Vector3 &, const std::vector<tf2::Vector3> &);
    DWA(ros::NodeHandle &, ros::NodeHandle &, std::string &);
    ~DWA();
};



#endif