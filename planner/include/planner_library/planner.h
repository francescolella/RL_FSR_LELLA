#include "sensor_msgs/LaserScan.h"
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32MultiArray.h"
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf_conversions/tf_eigen.h>
#include "geometry_msgs/Wrench.h"
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "boost/thread.hpp"
#include <vector>
#include <tf/tf.h>

#define drone_radius 0.5
using namespace std;
using namespace Eigen;

class PLAN
{
public:
    PLAN();
    void run();
    void laser_callback(sensor_msgs::LaserScan laser);
    void odom_callback(nav_msgs::OdometryConstPtr);
    void marker_cb(const geometry_msgs::PoseStamped &);
    void takeoff();
    void planning();
    void win_obstacle();
    void go_to_point(geometry_msgs::Vector3);
    Eigen::Vector3d repulsive_force();
    Eigen::Vector3d attractive_force(const Eigen::Vector3d &q_goal, const Eigen::Vector3d &q_curr, const Eigen::Vector3d &q_dot_curr);
    void calcolo_yaw_des();

private:
    ros::NodeHandle _nh;
    ros::Publisher _desider_pos_pub, _desider_yaw_pub, _desider_vel_pub, _desider_acc_pub, _position_marker_pub, _request_pub;
    ros::Subscriber _lidar_sub, _odom_sub, _marker_sub;

    tfScalar roll;
    tfScalar pitch;
    tfScalar yaw;

    bool _dest_near = false;
    bool TAKE_OFF = false;
    bool takeoff_started = false;
    float min_dist;
    float x_dist_near;
    float y_dist_near;
    bool win_obstacle_appr = false;
    bool win_obstacle_pas = false;
    bool next_aggiornamento = false;
    bool win_obstacle_appr_agg = false; // aggiunto per una prova, verificare se viene usato
    double gain_p;
    double gain_d;
    std_msgs::Float64 yaw_des;
    Eigen::Vector3d position_error = Eigen::MatrixXd::Zero(3, 1);
    Eigen::Vector3d window_center;
    Eigen::Vector3d via_point_pre_passaggio;
    Eigen::Vector3d via_point_post_passaggio;
    Eigen::Vector3d via_point_aggiornamento;
    geometry_msgs::Vector3 marker_pose;

    vector<Eigen::Vector3d> via_points;
    int curr_via_point_index;
    float marker_rot;

    geometry_msgs::Vector3 via_point_beckup;

    geometry_msgs::Vector3 pose_des;
    geometry_msgs::Vector3 velocity_des;
    geometry_msgs::Vector3 acceleration_des;

    geometry_msgs::Vector3 current_pose;
    geometry_msgs::Vector3 current_velocity;

    geometry_msgs::Vector3 via_point;
    geometry_msgs::Vector3 obs_point_near;
};