#include "controller_library/controller.h"

using namespace std;
using namespace Eigen;

// This function calculates the estimate of forces and torques

void IRIS_CTRL::estimate_force_torque()
{
	static integrator integ[6];
	VectorXd integ_in(6), integ_out(6);
	MatrixXd M_zita(6, 6), C_zita(6, 6);
	MatrixXd lambda(6, 4);

	VectorXd q(6), y(6);

	VectorXd zita_dot(6);
	VectorXd G_zita(6);
	Vector4d u;
	VectorXd F_hat(6);

	ros::Rate r(1000);
	while (!first_odom)
		usleep(100000);

	double start = ros::Time::now().toSec();
	double dur;
	while (ros::ok())
	{

		zita_dot << linear_vel.x, linear_vel.y, linear_vel.z,
			etha_dot;

		G_zita << -mass * acc_g * Vector3d::UnitZ(),
			Vector3d::Zero();

		u << mu_t,
			_tau_b;

		M_zita << mass * Matrix3d::Identity(), Matrix3d::Zero(),
			Matrix3d::Zero(), _M;

		C_zita << Matrix3d::Zero(), Matrix3d::Zero(),
			Matrix3d::Zero(), _C;

		lambda << -_Rb * Vector3d::UnitZ(), Matrix3d::Zero(),
			Vector3d::Zero(), _Q.transpose();

		q = M_zita * zita_dot;

		// F_hat= k_0*q - integr(C_0*F_hat + k_0*traspose(C_zita)*zita_dot + K_0*lambda*u - K_0*G_zita)=
		//    	= k_0*q - integr(C_0*F_hat + k_0*y)=
		//		= k_0*q - integr(integ_in)=
		//      = k_0*q - integ_out

		y = C_zita.transpose() * zita_dot + lambda * u - G_zita;

		integ_in = K_0 * y + C_0 * F_hat;
		for (int i = 0; i < 6; i++)
		{
			integ_out(i) = integ[i].update_value(integ_in(i));
		}

		F_hat = K_0 * q - integ_out;

		_est_force = F_hat.head(3);
		_est_torque = F_hat.tail(3);

		geometry_msgs::Vector3 _v3;
		_v3.x = _est_force(0);
		_v3.y = _est_force(1);
		_v3.z = _est_force(2);
		_force_stimate_pub.publish(_v3);
		_v3.x = _est_torque(0);
		_v3.y = _est_torque(1);
		_v3.z = _est_torque(2);
		_torque_stimate_pub.publish(_v3);

		if (!est_ready)
		{
			dur = ros::Time::now().toSec() - start;
			if (dur > 2.0)
			{
				est_ready = true;
				cout << "Estimation ready\n";
			}
		}
		r.sleep();
	}
}
