#include <ros/ros.h>
#include <dwa.h>

DWA::DWA(ros::NodeHandle &nh, ros::NodeHandle &nh_global, std::string &vehicle_name) : 
    vehicle_name_(vehicle_name),
    nh_(nh),
    nh_global_(nh_global)
{
    int control_frequency;
	nh_global_.getParamCached("control_frequency", control_frequency);
    vel_sub_ = nh_.subscribe("mavros/local_position/velocity_local", 10, &DWA::velCB, this);
    dt_ = 1.0/control_frequency;
}

DWA::~DWA(){
    vel_sub_.shutdown();
}

void DWA::velCB(const geometry_msgs::TwistStamped::ConstPtr &msg){
    cur_vel_.setX(msg->twist.linear.x);
    cur_vel_.setY(msg->twist.linear.y);
    cur_vel_.setZ(msg->twist.linear.z);
}

DynamicWindow DWA::calDynamicWindow(){
    float max_speed_;
	nh_global_.getParamCached("local_plan/max_speed", max_speed_);
    //로봇의 최대/최소 속도 범위
    tf2::Vector3 vs_max(max_speed_, max_speed_, max_speed_), vs_min(-max_speed_, -max_speed_, -1);
    tf2::Vector3 vd_max(cur_vel_.getX() + 3*dt_, cur_vel_.getY() + 3*dt_, cur_vel_.getZ() + 2*dt_);
    tf2::Vector3 vd_min(cur_vel_.getX() - 3*dt_, cur_vel_.getY() - 3*dt_, cur_vel_.getZ() - 1*dt_);

    DynamicWindow dw;
    dw.vx_max_ = std::min(vs_max.getX(), vd_max.getX());
    dw.vx_min_ = std::max(vs_min.getX(), vd_min.getX());
    dw.vy_max_ = std::min(vs_max.getY(), vd_max.getY());
    dw.vy_min_ = std::max(vs_min.getY(), vd_min.getY());
    dw.vz_max_ = std::min(vs_max.getZ(), vd_max.getZ());
    dw.vz_min_ = std::max(vs_min.getZ(), vd_min.getZ());

    return dw;
}

std::vector<tf2::Vector3> DWA::calTrjectory(const tf2::Vector3 &init_pose, const tf2::Vector3 &vel){
    double time = 0;
    double predict_time;
    nh_global_.getParamCached("dwa/prediction_time", predict_time);
    std::vector<tf2::Vector3> trajectory;
    tf2::Vector3 next_pose = init_pose;
    trajectory.push_back(init_pose);

    while ((time <= predict_time)) {
        next_pose += dt_*vel;
        trajectory.push_back(next_pose);
        time += dt_;
    }
    return trajectory;
    
}

tf2::Vector3 DWA::evalTrajectory(const tf2::Vector3 &init_pose, const tf2::Vector3 &goal, const std::vector<tf2::Vector3> &obs){
    DynamicWindow dw = calDynamicWindow();
    double vel_resol, to_goal_cost_gain, speed_cost_gain, max_speed, obstacle_cost_gain;
    nh_global_.getParamCached("local_plan/max_speed", max_speed);
    nh_global_.getParamCached("dwa/velocity_resolution", vel_resol);
    nh_global_.getParamCached("dwa/to_goal_cost_gain", to_goal_cost_gain);
    nh_global_.getParamCached("dwa/speed_cost_gain", speed_cost_gain);
    nh_global_.getParamCached("dwa/obstacle_cost_gain", obstacle_cost_gain);
    double min_cost = std::numeric_limits<double>::infinity();
    tf2::Vector3 setpoint_velocity(0, 0, 0);
    
    for (double vx = dw.vx_min_; vx <= dw.vx_max_; vx += vel_resol){
        for (double vy = dw.vy_min_; vy <= dw.vy_max_; vy += vel_resol){
            for (double vz = dw.vz_min_; vz <= dw.vz_max_; vz += vel_resol){
                tf2::Vector3 vel(vx, vy, vz);
                std::vector<tf2::Vector3> trajectory = calTrjectory(init_pose, vel);
                
                double to_goal_cost = to_goal_cost_gain * calToGoalCost(trajectory, goal);
                double speed_cost = 0;
                // double speed_cost = speed_cost_gain * (max_speed - vel.length());
                double ob_cost = 0;
                // double ob_cost = obstacle_cost_gain * calObstacleCost(init_pose, trajectory, obs);

                double final_cost = to_goal_cost + speed_cost + ob_cost;

                if (min_cost >= final_cost){
                    min_cost = final_cost;
                    setpoint_velocity = vel;
                }

            }
        }
    }
    return setpoint_velocity;
}

double DWA::calToGoalCost(std::vector<tf2::Vector3> &trajectory,const tf2::Vector3 &goal){
    tf2::Vector3 diff = goal - trajectory.back();
    return diff.length();
}

double DWA::calObstacleCost(const tf2::Vector3 &init_pose, std::vector<tf2::Vector3> &trajectory, const std::vector<tf2::Vector3> &obs){
    double robot_radius;
    nh_global_.getParamCached("dwa/robot_radius", robot_radius);
    double min_r = std::numeric_limits<double>::infinity();
    for (const auto &traj_point : trajectory){
        for (const auto &obstacle : obs){
            double r = (traj_point - init_pose - obstacle).length();
            if ( r <= robot_radius){
                return std::numeric_limits<double>::infinity();
            }
            if (min_r >= r){
                min_r = r;
            }
        }
    }
    return 1.0 / min_r;
}