#ifndef MY_LIBRARY

#define ixx 0.02941625
#define iyy 0.02941625
#define izz 0.05577725
// #define mass 1.515 //real
#define mass 1.3635 // underestimated
#define PI 3.14
#define acc_g 9.81

#define _l0_x 0.13
#define _l0_y 0.22
#define _l1_x 0.13
#define _l1_y 0.20
#define _l2_x 0.13
#define _l2_y 0.22
#define _l3_x 0.13
#define _l3_y 0.20

#define _c_T 8.55e-06
#define _c_a 1.75e-06

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <stdlib.h>
#include <cstdlib>
#include <cfloat>
#include <float.h>
#include <assert.h>
#include "controller_library/deriv_and_filter.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <eigen_conversions/eigen_msg.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Wrench.h"
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "tf_conversions/tf_eigen.h" //Usato per passare da tf a Eigen
#include "tf/tf.h"					 // Usato in odometry_cb() per passare da quaternione a angoli Eulero

class IRIS_CTRL
{
public:
	IRIS_CTRL();
	void key_input();
	void run();
	void vel_ctrl();
	void allocation();
	void odom_cb(nav_msgs::OdometryConstPtr);
	void desiderata_pos_cb(geometry_msgs::Vector3ConstPtr);
	void desiderata_yaw_cb(std_msgs::Float64ConstPtr);
	void desiderata_vel_cb(geometry_msgs::Vector3ConstPtr);
	void desiderata_acc_cb(geometry_msgs::Vector3ConstPtr);
	void stop_execution();
	void tau_b_cb(nav_msgs::OdometryConstPtr);
	void allocation_cb(geometry_msgs::Vector3ConstPtr);
	void calcolo_mu_t();
	void calcolo_error_pose();
	void calcolo_error_etha();
	void find_matrix();
	void estimate_force_torque();
	void request_cb(std_msgs::StringConstPtr);

private:
	ros::NodeHandle _nh;
	ros::Publisher motor_vel_pub, _tau_b_pub, _thrust_attitude_pub, _etha_des_pub, _force_stimate_pub, _torque_stimate_pub, _rpy_pub, _error_pose_pub;
	ros::Subscriber _odom_sub, _alloc_sub, _desid_pos_sub, _desid_yaw_sub, _desid_vel_sub, _desid_acc_sub, _request_sub;

	bool next_odom = true;
	Eigen::Vector4d w2;
	geometry_msgs::Point pose;
	geometry_msgs::Vector3 linear_vel;
	geometry_msgs::Vector3 tau_sig;
	geometry_msgs::Vector3 mu_d;
	geometry_msgs::Vector3 pose_des;
	geometry_msgs::Vector3 linear_vel_des;
	geometry_msgs::Vector3 linear_ac_des;

	Eigen::Vector3d etha;
	Eigen::Vector3d etha_dot;
	Eigen::Vector3d etha_des;
	Eigen::Vector3d etha_dot_des;
	Eigen::Vector3d etha_dot2_des;
	geometry_msgs::Vector3 error_pose;
	geometry_msgs::Vector3 error_linear_vel;
	geometry_msgs::Vector3 error_etha;
	geometry_msgs::Vector3 error_etha_dot;
	Eigen::Vector3d _est_force, _est_torque; // Estimated force and torque
	double K_0, C_0;
	bool est_ready;
	bool first_odom;

	Eigen::Vector3d _wb;
	Eigen::Vector3d _wbb;
	Eigen::Matrix3d _Q;
	Eigen::Matrix3d _Qdot;
	Eigen::Matrix3d _S;
	Eigen::Matrix3d _Ib;
	Eigen::Matrix3d _C;
	Eigen::Matrix3d _M;
	Eigen::Matrix3d _Rb;
	Eigen::Vector3d _tau_b;
	Eigen::Vector3d _tau_tilde;

	low_pass_filter roll_filter;
	low_pass_filter pitch_filter;

	integrator error_pose_x_integrator;
	integrator error_pose_y_integrator;
	integrator error_pose_z_integrator;

	angle_derivator des_roll_derivator;
	angle_derivator des_pitch_derivator;

	angle_double_derivator des_roll_doubleDerivator;
	angle_double_derivator des_pitch_doubleDerivator;

	// Kp and Ke
	double Ke00;
	double Ke11;
	double Ke22;
	double Ke03;
	double Ke14;
	double Ke25;
	double Kp00;
	double Kp11;
	double Kp22;
	double Kp03;
	double Kp14;
	double Kp25;

	double Ki_x;
	double Ki_y;
	double Ki_z;

	float _vel[4];
	float mu_t;

	bool go;
};

#endif
