#ifndef ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_NODE_H
#define ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>
#include <random>
#include <deque>
#include <time.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/RateThrust.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "rotors_control/common.h"
#include "rotors_control/roll_pitch_yawrate_thrust_controller.h"
#include "rotors_control/pid.h"

namespace rotors_control
{

  class AccCommandConverterNode
  {
  public:
    AccCommandConverterNode();
    ~AccCommandConverterNode();

  private:
    inline static constexpr double kGravity = 9.8066;

    bool use_vehicle_frame;
    bool receive_first_odom;
    bool receive_thrust_cmd;
    bool receive_goal;
    bool receive_goal_training;
    bool use_yaw_stabilize;
    mav_msgs::EigenOdometry odometry;
    mav_msgs::RateThrust rate_thrust_cmd;
    mav_msgs::EigenOdometry goal_odometry, goal_training_odometry;
    double goal_yaw, goal_training_yaw;
    std::string frame_id, vehicle_frame_id;
    double K_yaw;
    double yaw_rate_limit;
    double Kp_x, Ki_x, Kd_x, acc_x_max, alpha_x;
    double Kp_y, Ki_y, Kd_y, acc_y_max, alpha_y;
    double Kp_z, Ki_z, Kd_z, acc_z_max, alpha_z;
    double odom_dtime;
    double eps_explore;
    double noise_x, noise_y, noise_z;
    std::random_device rd_x, rd_y, rd_z;
    std::mt19937 gen_x, gen_y, gen_z;
    std::normal_distribution<> d_x, d_y, d_z;
    PID *pid_x;
    PID *pid_y;
    PID *pid_z;

    double mass;

    // subscribers
    ros::Subscriber cmd_rate_thrust_sub_;
    ros::Subscriber odometry_sub_;
    ros::Subscriber goal_pose_sub_;
    ros::Subscriber goal_training_pose_sub_;

    ros::Publisher cmd_roll_pitch_yawrate_thrust_pub_;
    ros::Publisher state_action_pub_;

    void RateThrustCallback(const mav_msgs::RateThrustPtr &rate_thrust_msg);

    void OdometryCallback(const nav_msgs::OdometryConstPtr &odometry_msg);

    // void GoalPoseCallback(const geometry_msgs::Pose& goal_msg);
    void GoalPoseCallback(const geometry_msgs::PoseStamped &goal_msg);

    void GoalTrainingPoseCallback(const geometry_msgs::Pose &goal);

    void convertGoal2WorldFrame(geometry_msgs::Pose &goal,
                                mav_msgs::EigenOdometry &robot_odom,
                                mav_msgs::EigenOdometry *goal_in_world);

    void convertGoal2VehicleFrame(mav_msgs::EigenOdometry &goal_odom,
                                  mav_msgs::EigenOdometry &robot_odom,
                                  nav_msgs::Odometry *goal_in_vehicle_frame);
  };
}

#endif // ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_NODE_H