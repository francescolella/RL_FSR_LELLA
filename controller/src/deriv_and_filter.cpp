#include "controller_library/deriv_and_filter.h"

discr_system::discr_system(Eigen::MatrixXd A,	Eigen::MatrixXd B,	Eigen::MatrixXd C,	Eigen::MatrixXd D,	Eigen::VectorXd x_0)
{
	if (A.rows() != B.rows())
		ROS_ERROR("size of A do not match size of B ");
	if (C.rows() != D.rows())
		ROS_ERROR("size of C do not match size of D ");
	if (A.cols() != C.cols())
		ROS_ERROR("size of A do not match size of C ");
	if (B.cols() != D.cols())
		ROS_ERROR("size of B do not match size of D ");
	if (A.cols() != x_0.size())
		ROS_ERROR("size of x_0 do not match num. of columns of A");

	_A = A;
	_B = B;
	_C = C;
	_D = D;

	_x = x_0;
}

Eigen::VectorXd discr_system::compute_next_value(const Eigen::VectorXd &u)
{
	Eigen::VectorXd y = _C * _x + _D * u;
	_x = _A * _x + _B * u;
	return y;
}

double discr_system::compute_next_value_double(const double &u)
{

	Eigen::VectorXd y = _C * _x + _D * u;
	_x = _A * _x + _B * u;
	return (double)y(0);
}

void discr_system::set_matrix(	Eigen::MatrixXd A,	Eigen::MatrixXd B,	Eigen::MatrixXd C,	Eigen::MatrixXd D,	Eigen::VectorXd x_0)
{

	if (A.rows() != B.rows())
		ROS_ERROR("size of A do not match size of B ");
	if (C.rows() != D.rows())
		ROS_ERROR("size of C do not match size of D ");
	if (A.cols() != C.cols())
		ROS_ERROR("size of A do not match size of C ");
	if (B.cols() != D.cols())
		ROS_ERROR("size of B do not match size of D ");
	if (A.cols() != x_0.size())
		ROS_ERROR("size of x_0 do not match num. of columns of A");

	_A = A;
	_B = B;
	_C = C;
	_D = D;

	_x = x_0;
}

//     angle_derivator functions
// series of 5-th order Butt, discetized with Ts=0.001, with omega w_c=100 and a derivator (z-1)/(Ts*z)

angle_derivator::angle_derivator()
{
	// define system's matrices
	Eigen::MatrixXd A;
	A.resize(6, 6);
	A << 4.67643242525, -2.18936515142, 2.05218724215, -0.962785833171, 0.3617070018, 0,
          4.0,              0,             0,               0,            0, 0,
            0,            1.0,             0,               0,            0, 0,
            0,              0,           1.0,               0,            0, 0,
            0,              0,             0,             0.5,            0, 0,
            0,              0,             0,               0,          1.0, 0;

	Eigen::MatrixXd B;
	B.resize(6, 1);
	B << 0.0625,
		0,
		0,
		0,
		0,
		0;

	Eigen::MatrixXd C;
	C.resize(1, 6);
	C << 0.0370523079935, -0.00401151593223, 0.00876377179349, -0.00944960073402, -0.00699625534756, -0.00213522714046;
	Eigen::MatrixXd D;
	D.resize(1, 1);
	D << 0.000266903392557;

	//	cout<<"A: "<<A<<endl<<"B: "<<B<<endl<<"C: "<<C<<endl<<"D: "<<D<<endl;
	Eigen::VectorXd x_0(A.cols());
	x_0.setZero();

	set_matrix(A, B, C, D, x_0);
}


// 	double derivator functions
// series of last derivator with a derivator (z-1)/(Ts*z)
angle_double_derivator::angle_double_derivator()
{
	// define system's matrices
	Eigen::MatrixXd A;
	A.resize(7, 7);

	A << 4.67643242525, -2.18936515142, 2.05218724215, -0.962785833171, 0.3617070018,   0, 0,
          4.0,              0,             0,               0,            0,   0, 0,
            0,            1.0,             0,               0,            0,   0, 0,
            0,              0,           1.0,               0,            0,   0, 0,
            0,              0,             0,             0.5,            0,   0, 0,
            0,              0,             0,               0,          1.0,   0, 0,
            0,              0,             0,               0,            0, 1.0, 0;

	Eigen::MatrixXd B;
	B.resize(7, 1);
	B << 2,
		0,
		0,
		0,
		0,
		0,
		0;

	Eigen::MatrixXd C;
	C.resize(1, 7);

	C << 1.02443292852, -0.258811569161, 0.107053248198, -0.295300022938, 0.114996261085, 0.200177544418, 0.0667258481394;

	Eigen::MatrixXd D;
	D.resize(1, 1);
	D << 0.266903392557;

	//	cout<<"A: "<<A<<endl<<"B: "<<B<<endl<<"C: "<<C<<endl<<"D: "<<D<<endl;
	Eigen::VectorXd x_0(A.cols());
	x_0.setZero();

	set_matrix(A, B, C, D, x_0);
}


// 	low_pass_filter functions
// use 5-th order Butt, discetized with Ts=0.001, with cut omega wc=50 rad/s

low_pass_filter::low_pass_filter()
{
	// define system's matrices
	Eigen::MatrixXd A;
	A.resize(5, 5);

	A << 4.83820151539, -2.34145455299, 1.13345043378, -0.548822219545, 0.425295347102,
          4.0,              0,             0,               0,              0,
            0,            2.0,             0,               0,              0,
            0,              0,           1.0,               0,              0,
            0,              0,             0,            0.25,              0;

	Eigen::MatrixXd B;
	B.resize(5, 1);
	B << 0.00048828125,
		0,
		0,
		0,
		0;

	Eigen::MatrixXd C;
	C.resize(1, 5);

	C << 0.000181659652552, 0.00000292749754849, 0.0000440097488343, 0.00000140660151227, 0.0000170853210314;

	Eigen::MatrixXd D;
	D.resize(1, 1);
	D << 0.00000000901597736982;

	//	cout<<"A: "<<A<<endl<<"B: "<<B<<endl<<"C: "<<C<<endl<<"D: "<<D<<endl;
	Eigen::VectorXd x_0(A.cols());
	x_0.setZero();

	set_matrix(A, B, C, D, x_0);
}


//  Integral functions
// A discrete Integral with Ts=0.001 (Ts)/(z-1)

integrator::integrator()
{

	// define system's matrices
	Eigen::MatrixXd A;
	A.resize(1, 1);

	A << 1;

	Eigen::MatrixXd B;
	B.resize(1, 1);
	B << 1;

	Eigen::MatrixXd C;
	C.resize(1, 1);

	C << 0.001;

	Eigen::MatrixXd D;
	D.resize(1, 1);
	D << 0;

	//	cout<<"A: "<<A<<endl<<"B: "<<B<<endl<<"C: "<<C<<endl<<"D: "<<D<<endl;
	Eigen::VectorXd x_0(A.cols());
	x_0.setZero();

	set_matrix(A, B, C, D, x_0);
}

double low_pass_filter::update_value(const double &u)
{
	Eigen::VectorXd u_(1);
	u_(0) = u;
	u_ = compute_next_value(u_);
	return (double)u_(0);
}

double angle_derivator::update_value(const double &u)
{
	Eigen::VectorXd u_(1);
	u_(0) = u;
	u_ = compute_next_value(u_);
	return (double)u_(0);
}

double integrator::update_value(const double &u)
{
	Eigen::VectorXd u_(1);
	u_(0) = u;
	u_ = compute_next_value(u_);
	return (double)u_(0);
}

double angle_double_derivator::update_value(const double &u)
{
	Eigen::VectorXd u_(1);
	u_(0) = u;
	u_ = compute_next_value(u_);
	return (double)u_(0);
}