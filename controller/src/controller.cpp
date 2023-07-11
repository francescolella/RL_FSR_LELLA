#define _USE_MATH_DEFINES

#include "controller_library/controller.h"

#define vel_lim 1080 // propeller speed limit

#include "boost/thread.hpp"
#include <cmath>

#include <iostream>

using namespace std;
using namespace Eigen;

IRIS_CTRL::IRIS_CTRL()
{
	// pubblishers and subscribers inizializzation
	// pub
	motor_vel_pub = _nh.advertise<std_msgs::Float32MultiArray>("iris/cmd/motor_vel", 0);	// publisher delle velocità dei motori
	_etha_des_pub = _nh.advertise<geometry_msgs::Vector3>("/iris/etha_des", 0);				// publisher di etha des
	_tau_b_pub = _nh.advertise<geometry_msgs::Vector3>("/iris/tau_b", 0);					// pablisher di tua_b
	_force_stimate_pub = _nh.advertise<geometry_msgs::Vector3>("/iris/force_stimate", 0);	// il publisher delle forze stimate
	_torque_stimate_pub = _nh.advertise<geometry_msgs::Vector3>("/iris/torque_stimate", 0); // il publisher delle torque stimate
	_rpy_pub = _nh.advertise<geometry_msgs::Vector3>("/iris/rpy", 0);						// il publisheer dell'orientamento RPY
	_error_pose_pub = _nh.advertise<geometry_msgs::Vector3>("/iris/error_pose", 0);			// il publisheer dell'orientamento RPY

	// sub
	_odom_sub = _nh.subscribe("/iris/odometry", 0, &IRIS_CTRL::odom_cb, this);					  // subscriber per la callback dell'odometry
	_alloc_sub = _nh.subscribe("/iris/tau_b", 0, &IRIS_CTRL::allocation_cb, this);				  // subscriber per la generazione della matrice di allocazione
	_desid_pos_sub = _nh.subscribe("iris/desideratapos", 0, &IRIS_CTRL::desiderata_pos_cb, this); // subscriber per l'acquisizione della posizione desiderata  dal planner
	_desid_yaw_sub = _nh.subscribe("iris/desideratayaw", 0, &IRIS_CTRL::desiderata_yaw_cb, this); // subscriber per l'acquisizione della yaw desiderata  dal planner
	_desid_vel_sub = _nh.subscribe("iris/desideratavel", 0, &IRIS_CTRL::desiderata_vel_cb, this); // subscriber per l'acquisizione della velocità desiderata  dal planner
	_request_sub = _nh.subscribe("iris/request", 0, &IRIS_CTRL::request_cb, this);				  // subscriber per l'acquisizione delle richieste provenienti dal planner
	_desid_acc_sub = _nh.subscribe("iris/desiderataacc", 0, &IRIS_CTRL::desiderata_acc_cb, this); // subscriber per l'acquisizione dell'accelerazione desiderata dal planner

	// parameter acquisitions
	if (_nh.hasParam("Kp00")){_nh.getParam("Kp00", Kp00);ROS_INFO_STREAM("_Kp00\t" << Kp00);}
	if (_nh.hasParam("Kp11")){_nh.getParam("Kp11", Kp11);ROS_INFO_STREAM("_Kp11\t" << Kp11);}
	if (_nh.hasParam("Kp22")){_nh.getParam("Kp22", Kp22);ROS_INFO_STREAM("_Kp22\t" << Kp22);}
	if (_nh.hasParam("Kp03")){_nh.getParam("Kp03", Kp03);ROS_INFO_STREAM("_Kp03\t" << Kp03);}
	if (_nh.hasParam("Kp14")){_nh.getParam("Kp14", Kp14);ROS_INFO_STREAM("_Kp14\t" << Kp14);}
	if (_nh.hasParam("Kp25")){_nh.getParam("Kp25", Kp25);ROS_INFO_STREAM("_Kp25\t" << Kp25);}
	if (_nh.hasParam("Ke00")){_nh.getParam("Ke00", Ke00);ROS_INFO_STREAM("_Ke00\t" << Ke00);}
	if (_nh.hasParam("Ke11")){_nh.getParam("Ke11", Ke11);ROS_INFO_STREAM("_Ke11\t" << Ke11);}
	if (_nh.hasParam("Ke22")){_nh.getParam("Ke22", Ke22);ROS_INFO_STREAM("_Ke22\t" << Ke22);}
	if (_nh.hasParam("Ke03")){_nh.getParam("Ke03", Ke03);ROS_INFO_STREAM("_Ke03\t" << Ke03);}
	if (_nh.hasParam("Ke14")){_nh.getParam("Ke14", Ke14);ROS_INFO_STREAM("_Ke14\t" << Ke14);}
	if (_nh.hasParam("Ke25")){_nh.getParam("Ke25", Ke25);ROS_INFO_STREAM("_Ke25\t" << Ke25);}
	if (_nh.hasParam("Ki_x")){_nh.getParam("Ki_x", Ki_x);ROS_INFO_STREAM("_Ki_x\t" << Ki_x);}
	if (_nh.hasParam("Ki_y")){_nh.getParam("Ki_y", Ki_y);ROS_INFO_STREAM("_Ki_y\t" << Ki_y);}
	if (_nh.hasParam("Ki_z")){_nh.getParam("Ki_z", Ki_z);ROS_INFO_STREAM("_Ki_z\t" << Ki_z);}
	if (_nh.hasParam("K_0")){_nh.getParam("K_0", K_0);ROS_INFO_STREAM("K_0\t" << K_0);}
	if (_nh.hasParam("C_0")){_nh.getParam("C_0", C_0);ROS_INFO_STREAM("C_0\t" << C_0);}

	// Inertia Matrix
	_Ib << ixx, 0, 0,
		0, iyy, 0,
		0, 0, izz;

	first_odom = false; // boolean variable for the first odometry
	est_ready = false;	// boolean variable for the first estimation

	// variables inizializzation
	for (int i = 0; i < 4; i++)
		_vel[0] = 0.0;
	tau_sig.x = 0.0;
	tau_sig.y = 0.0;
	tau_sig.z = 0.0;

	go = true;

	mu_d.x = 0.0;
	mu_d.y = 0.0;
	mu_d.z = 0.0;

	linear_vel_des.x = 0.0;
	linear_vel_des.y = 0.0;
	linear_vel_des.z = 0.0;

	linear_ac_des.x = 0.0;
	linear_ac_des.y = 0.0;
	linear_ac_des.z = 0.0;

	etha_des(0) = 0;
	etha_des(1) = 0;
	etha_des(2) = 0;

	error_pose.x = 0.0;
	error_pose.y = 0.0;
	error_pose.z = 0.0;

	error_linear_vel.x = 0.0;
	error_linear_vel.y = 0.0;
	error_linear_vel.z = 0.0;
}

// This function waits for the landing requeste from the planner
void IRIS_CTRL::request_cb(std_msgs::StringConstPtr msg)
{

	if (msg->data == "land") 
		stop_execution();
}

// This function take desiderata position from the planner subscriber
void IRIS_CTRL::desiderata_pos_cb(geometry_msgs::Vector3ConstPtr desiderata_pose)
{
	pose_des.x = desiderata_pose->x;
	pose_des.y = desiderata_pose->y;
	pose_des.z = desiderata_pose->z;

	// only for debug
	geometry_msgs::Vector3 _v3;
	_v3.x = pose.x - pose_des.x;
	_v3.y = pose.y - pose_des.y;
	_v3.z = pose.z - pose_des.z;
	_error_pose_pub.publish(_v3);
}

// This function take desiderata velocity from the planner subscriber
void IRIS_CTRL::desiderata_vel_cb(geometry_msgs::Vector3ConstPtr desiderata_vel)
{
	linear_vel_des.x = desiderata_vel->x;
	linear_vel_des.y = desiderata_vel->y;
	linear_vel_des.z = desiderata_vel->z;
}

// This function take desiderata acceleration from the planner subscriber
void IRIS_CTRL::desiderata_acc_cb(geometry_msgs::Vector3ConstPtr desiderata_acc)
{
	linear_ac_des.x = desiderata_acc->x;
	linear_ac_des.y = desiderata_acc->y;
	linear_ac_des.z = desiderata_acc->z;
}

// This function take desiderata yaw from the planner subscriber
void IRIS_CTRL::desiderata_yaw_cb(std_msgs::Float64ConstPtr desiderata_yaw)
{
	etha_des(2) = desiderata_yaw->data;

}

// This function take the odometry from the subscriber (Gazebo)
// and do the necessary operations
void IRIS_CTRL::odom_cb(nav_msgs::OdometryConstPtr odom)
{

	tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
	tf::Matrix3x3 Rb(q); // make rotation matrix from the quaternion

	pose.x = odom->pose.pose.position.x;
	pose.y = odom->pose.pose.position.y;
	pose.z = odom->pose.pose.position.z;

	if (!first_odom)
	{
		pose_des.z = pose.z;
		pose_des.y = pose.y;
		pose_des.x = pose.x;
	}

	linear_vel.x = odom->twist.twist.linear.x;
	linear_vel.y = odom->twist.twist.linear.y;
	linear_vel.z = odom->twist.twist.linear.z;

	_wb(0) = odom->twist.twist.angular.x;
	_wb(1) = odom->twist.twist.angular.y;
	_wb(2) = odom->twist.twist.angular.z;

	Rb.getRPY(etha(0), etha(1), etha(2)); // transform rotation matrix in roll, pitch and yaw angles
	first_odom = true;

	// only for debug
	geometry_msgs::Vector3 _v3;
	_v3.x = etha(0);
	_v3.y = etha(1);
	_v3.z = etha(2);
	_rpy_pub.publish(_v3);

}

void IRIS_CTRL::find_matrix()
{

	// ROTATION MATRIX

	_Rb << cos(etha(1)) * cos(etha(2)), sin(etha(0)) * sin(etha(1)) * cos(etha(2)) - cos(etha(0)) * sin(etha(2)), cos(etha(0)) * sin(etha(1)) * cos(etha(2)) + sin(etha(0)) * sin(etha(2)),
		cos(etha(1)) * sin(etha(2)), sin(etha(0)) * sin(etha(1)) * sin(etha(2)) + cos(etha(0)) * cos(etha(2)), cos(etha(0)) * sin(etha(1)) * sin(etha(2)) - sin(etha(0)) * cos(etha(2)),
		-sin(etha(1)), sin(etha(0)) * cos(etha(1)), cos(etha(0)) * cos(etha(1));
	_wbb = _Rb.transpose() * _wb;

	_Q << 1, 0, -sin(etha(1)),
		0, cos(etha(0)), cos(etha(1)) * sin(etha(0)),
		0, -sin(etha(0)), cos(etha(1)) * cos(etha(0));

	// compute current rotation velocity
	etha_dot = _Q.inverse() * _wbb;

	_Qdot << 0, 0, -etha_dot(1) * cos(etha(1)),
		0, -etha_dot(0) * sin(etha(0)), -etha_dot(1) * sin(etha(1)) * sin(etha(0)) + cos(etha(1)) * etha_dot(0) * cos(etha(0)),
		0, -etha_dot(0) * cos(etha(0)), -etha_dot(1) * sin(etha(1)) * cos(etha(0)) - cos(etha(1)) * etha_dot(0) * sin(etha(0));

	_M = _Q.transpose() * _Ib * _Q;

	_S << 0, -_wbb(2), _wbb(1),
		_wbb(2), 0, -_wbb(0),
		-_wbb(1), _wbb(0), 0;

	_C = _Q.transpose() * _S * _Ib * _Q + _Q.transpose() * _Ib * _Qdot;
	_tau_tilde << tau_sig.x, tau_sig.y, tau_sig.z;
}

// This function implement the key_input with the graphical text 
void IRIS_CTRL::key_input()
{

	string input;

	while (ros::ok())
	{
		cout << "Keyboard Input: " << endl;
		cout << "[x]: modificare pose x" << endl;
		cout << "[y]: modificare pose y" << endl;
		cout << "[z]: modificare pose z" << endl;
		cout << "[r]: per ruotare" << endl;
		cout << "[f]: stop motori" << endl;
		cout << "[t] takoff" << endl;
		cout << " Posizione desiderata attuale: " << pose_des.x << ",  " << pose_des.y << ",  " << pose_des.z << endl;
		getline(cin, input);

		if (input == "x")
		{
			go = true;
			cout << "p per aumentare e m per diminuire la posizione x desiderata; attuale: " << pose.x << endl;
			getline(cin, input);
			if (input == "p")
				pose_des.x += 0.05;
			else
				pose_des.x -= 0.05;
			cout << endl
				 << "----NEXT----" << endl;
		}
		else if (input == "y")
		{
			go = true;
			cout << "p per aumentare e m per diminuire la posizione y desiderata; attuale: " << pose.y << endl;
			getline(cin, input);
			if (input == "p")
				pose_des.y += 0.05;
			else
				pose_des.y -= 0.05;
			cout << endl
				 << "----NEXT----" << endl;
		}
		else if (input == "z")
		{
			go = true;
			cout << "p per aumentare e m per diminuire la posizione z desiderata; attuale: " << pose.z << endl;
			getline(cin, input);
			if (input == "p")
				pose_des.z += 0.25;
			else
				pose_des.z -= 0.25;
			cout << endl
				 << "----NEXT----" << endl;
		}
		else if (input == "r")
		{
			go = true;
			cout << "p per aumentare e m per ruotare attorno l'asse z desiderata; attuale: " << etha(2) << endl;
			getline(cin, input);
			if (input == "p")
				etha_des(2) += 0.174533;
			else
				etha_des(2) -= 0.174533;
			cout << endl
				 << "----NEXT----" << endl;
		}
		else if (input == "t")
		{
			pose_des.x = 0;
			pose_des.y = 0;
			pose_des.z = -2.5;
			etha_des(2) = 0;
		}

		else if (input == "f")
		{
			stop_execution();
		}
	}
}

void IRIS_CTRL::stop_execution()
{
	_vel[0] = _vel[1] = _vel[2] = _vel[3] = 0.0;
	go = false;
}

void IRIS_CTRL::vel_ctrl()
{

	while (!first_odom || !est_ready)
		usleep(100000);

	geometry_msgs::Vector3 tau_b;

	ros::Rate r(1000);

	std_msgs::Float32MultiArray m_data;
	m_data.data.resize(4);

	while (ros::ok())
	{

		find_matrix(); // call the function for compute all necessary matrixs
		calcolo_mu_t(); // call the function for compute mu_t

		_tau_b = _Ib * _Q * _tau_tilde + _Q.inverse() * _C * etha_dot;

		tau_b.x = _tau_b(0);
		tau_b.y = _tau_b(1);
		tau_b.z = _tau_b(2);
		_tau_b_pub.publish(tau_b);

		for (int i = 0; i < 4; i++)
		{
			m_data.data[i] = _vel[i];
			if (std::isnan(m_data.data[i]))
			{
				m_data.data[i] = 0;
			}
		}

		motor_vel_pub.publish(m_data); // publish propeller speed
		r.sleep();
	}
}

// This function is called when tau_b is computed to compute the propeller speed by use of allocation matrix
void IRIS_CTRL::allocation_cb(geometry_msgs::Vector3ConstPtr tau_b)
{

	Vector4d ftau;
	ftau << mu_t, tau_b->x, tau_b->y, tau_b->z;
	Eigen::Matrix4d Gq;

	Gq << _c_T, _c_T, _c_T, _c_T,
		-_l0_y * _c_T, _l1_y * _c_T, _l2_y * _c_T, -_l3_y * _c_T,
		_l0_x * _c_T, -_l1_x * _c_T, _l2_x * _c_T, -_l3_x * _c_T,
		_c_a, _c_a, -_c_a, -_c_a;

	w2 = Gq.inverse() * ftau;
	if (w2(0) < 0)
		w2(0) = 0;
	if (w2(1) < 0)
		w2(1) = 0;
	if (w2(2) < 0)
		w2(2) = 0;
	if (w2(3) < 0)
		w2(3) = 0;

	if (go == true)
	{
		_vel[0] = sqrt(w2(0));
		_vel[1] = sqrt(w2(1));
		_vel[2] = sqrt(w2(2));
		_vel[3] = sqrt(w2(3));
	}

	if (std::isnan(_vel[0]) || std::isnan(_vel[1]) || std::isnan(_vel[2]) || std::isnan(_vel[3]))
	{
		cout << "velocità nan" << endl;
		cout << "vel(0)" << _vel[0] << endl;
		cout << "vel(1)" << _vel[1] << endl;
		cout << "vel(2)" << _vel[2] << endl;
		cout << "vel(3)" << _vel[3] << endl;
		_vel[0] = _vel[1] = _vel[2] = _vel[3] = 0;
	}
	else
	{
		// propeller velocity saturation
		if (_vel[0] > vel_lim)
		{
			//cout << "saturazione vel_0" << endl;
			_vel[0] = vel_lim;
		}
		if (_vel[1] > vel_lim)
		{
			//cout << "saturazione vel_1" << endl;
			_vel[1] = vel_lim;
		}
		if (_vel[2] > vel_lim)
		{
			//cout << "saturazione vel_2" << endl;
			_vel[2] = vel_lim;
		}
		if (_vel[3] > vel_lim)
		{
			//cout << "saturazione vel_3" << endl;
			_vel[3] = vel_lim;
		}
	}
}

// This function compute the total thrust
void IRIS_CTRL::calcolo_mu_t()
{
	float Kp[3][6] = {};
	float Ke[3][6] = {};

	Kp[0][0] = Kp00;
	Kp[1][1] = Kp11;
	Kp[2][2] = Kp22;
	Kp[0][3] = Kp03;
	Kp[1][4] = Kp14;
	Kp[2][5] = Kp25;

	Ke[0][0] = Ke00;
	Ke[1][1] = Ke11;
	Ke[2][2] = Ke22;
	Ke[0][3] = Ke03;
	Ke[1][4] = Ke14;
	Ke[2][5] = Ke25;

	calcolo_error_etha();

	tau_sig.x = -Ke[0][0] * error_etha.x - Ke[0][1] * error_etha.y - Ke[0][2] * error_etha.z - Ke[0][3] * error_etha_dot.x - Ke[0][4] * error_etha_dot.y - Ke[0][5] * error_etha_dot.z + etha_dot2_des(0) - _est_torque(0);
	tau_sig.y = -Ke[1][0] * error_etha.x - Ke[1][1] * error_etha.y - Ke[1][2] * error_etha.z - Ke[1][3] * error_etha_dot.x - Ke[1][4] * error_etha_dot.y - Ke[1][5] * error_etha_dot.z + etha_dot2_des(1) - _est_torque(1);
	tau_sig.z = -Ke[2][0] * error_etha.x - Ke[2][1] * error_etha.y - Ke[2][2] * error_etha.z - Ke[2][3] * error_etha_dot.x - Ke[2][4] * error_etha_dot.y - Ke[2][5] * error_etha_dot.z + etha_dot2_des(2) - _est_torque(2);

	calcolo_error_pose();

	mu_d.x = -Kp[0][0] * error_pose.x - Kp[0][3] * error_linear_vel.x - Ki_x * error_pose_x_integrator.update_value(error_pose.x) + linear_ac_des.x - _est_force(0) / mass;
	mu_d.y = -Kp[1][1] * error_pose.y - Kp[1][4] * error_linear_vel.y - Ki_y * error_pose_y_integrator.update_value(error_pose.y) + linear_ac_des.y - _est_force(1) / mass;
	mu_d.z = -Kp[2][2] * error_pose.z - Kp[2][5] * error_linear_vel.z - Ki_z * error_pose_z_integrator.update_value(error_pose.z) + linear_ac_des.z - _est_force(2) / mass;

	mu_t = mass * sqrt(mu_d.x * mu_d.x + mu_d.y * mu_d.y + (mu_d.z - acc_g) * (mu_d.z - acc_g));

	etha_des(0) = asin(mass * (mu_d.y * cos(etha_des(2)) - mu_d.x * sin(etha_des(2))) / mu_t);		// roll desiderata
	etha_des(1) = atan((mu_d.x * cos(etha_des(2)) + mu_d.y * sin(etha_des(2))) / (mu_d.z - acc_g)); // pitch desiderata

	// roll and pitch filter
	etha_des(0) = roll_filter.update_value(etha_des(0));
	etha_des(1) = pitch_filter.update_value(etha_des(1));

	// only for debug
	geometry_msgs::Vector3 etha_des_deb;
	etha_des_deb.x = etha_des(0);
	etha_des_deb.y = etha_des(1);
	etha_des_deb.z = etha_des(2);
	_etha_des_pub.publish(etha_des_deb);
	//

	// compute numerical derivation to find the desired angular velocity
	etha_dot_des(0) = des_roll_derivator.update_value(etha_des(0));
	etha_dot_des(1) = des_pitch_derivator.update_value(etha_des(1));
	etha_dot_des(2) = 0.0;

	//compute the double numerical derivation to find the desired angular acceleration
	etha_dot2_des(0) = des_roll_doubleDerivator.update_value(etha_des(0));
	etha_dot2_des(1) = des_pitch_doubleDerivator.update_value(etha_des(1));
	etha_dot2_des(2) = 0.0;

}

void IRIS_CTRL::calcolo_error_pose()
{

	error_pose.x = pose.x - pose_des.x;
	error_pose.y = pose.y - pose_des.y;
	error_pose.z = pose.z - pose_des.z;

	error_linear_vel.x = linear_vel.x - linear_vel_des.x;
	error_linear_vel.y = linear_vel.y - linear_vel_des.y;
	error_linear_vel.z = linear_vel.z - linear_vel_des.z;

}

void IRIS_CTRL::calcolo_error_etha()
{
	int div;
	float temp_1, temp_2;
	if (etha_des(2) > PI)
	{
		div = int(floor(etha_des(2) / PI)) % 2;
		if (div == 0)
			etha_des(2) = etha_des(2) - floor(etha_des(2) / PI) * PI;
		else
			etha_des(2) = etha_des(2) - (floor(etha_des(2) / PI) + 1) * PI;
	}
	else if (etha_des(2) < -PI)
	{
		div = int(floor(abs(etha_des(2)) / (PI))) % 2;
		if (div == 0)
			etha_des(2) = etha_des(2) + floor(abs(etha_des(2)) / PI) * PI;
		else
			etha_des(2) = etha_des(2) + (floor(abs(etha_des(2)) / PI) + 1) * PI;
	}
	error_etha.x = etha(0) - etha_des(0);
	error_etha.y = etha(1) - etha_des(1);
	error_etha.z = etha(2) - etha_des(2);
	if (error_etha.z > PI)
		error_etha.z = error_etha.z - 2 * PI;
	else if (error_etha.z < -PI)
		error_etha.z = error_etha.z + 2 * PI;

	error_etha_dot.x = etha_dot(0) - etha_dot_des(0);
	error_etha_dot.y = etha_dot(1) - etha_dot_des(1);
	error_etha_dot.z = etha_dot(2) - etha_dot_des(2);
}

void IRIS_CTRL::run()
{
	boost::thread key_input_t(&IRIS_CTRL::key_input, this);
	cout << "CONTROLLER STARTED" << endl;
	boost::thread vel_ctrl_t(&IRIS_CTRL::vel_ctrl, this);
	boost::thread estimator(&IRIS_CTRL::estimate_force_torque, this);

	ros::spin();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	IRIS_CTRL kc;
	kc.run();
	return 0;
}
