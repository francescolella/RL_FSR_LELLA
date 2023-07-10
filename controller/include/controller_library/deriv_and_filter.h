#ifndef MY_LIBRARY
#define MY_LIBRARY

#include <math.h>
#include "ros/ros.h"
#include <Eigen/Dense>

using namespace std;

/********************************************************
Class to implement a time discrete system
*********************************************************/

class discr_system
{
public:
	discr_system(){};
	discr_system(Eigen::MatrixXd A,	Eigen::MatrixXd B,	Eigen::MatrixXd C,	Eigen::MatrixXd D,	Eigen::VectorXd x_0);

	void set_matrix(Eigen::MatrixXd A,	Eigen::MatrixXd B,	Eigen::MatrixXd C,	Eigen::MatrixXd D,	Eigen::VectorXd x_0);

	Eigen::VectorXd compute_next_value(const Eigen::VectorXd &u);
	double compute_next_value_double(const double &u);

private:
	Eigen::MatrixXd _A;
	Eigen::MatrixXd _B;
	Eigen::MatrixXd _C;
	Eigen::MatrixXd _D;
	Eigen::VectorXd _x;
};

class angle_derivator : public discr_system
{
public:
	angle_derivator();
	double update_value(const double &u);
};

class angle_double_derivator : public discr_system
{
public:
	angle_double_derivator();
	double update_value(const double &u);
};

class low_pass_filter : public discr_system
{
public:
	low_pass_filter();
	double update_value(const double &u);
};

class integrator : public discr_system
{
public:
	integrator();
	double update_value(const double &u);
};

#endif
