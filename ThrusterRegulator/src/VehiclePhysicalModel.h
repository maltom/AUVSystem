#pragma once

#include <fstream>
#include <memory>
#include <utility>

#include <Eigen/Dense>

using namespace Eigen;

class VehiclePhysicalModel final
{
public:
	struct Inertial
	{
		double m{ 0.0 };
		double Ix{ 0.0 }, Iy{ 0.0 }, Iz{ 0.0 };
		double Ixy{ 0.0 }, Iyx{ 0.0 }, Ixz{ 0.0 }, Izx{ 0.0 }, Iyz{ 0.0 }, Izy{ 0.0 };
		Matrix3d Ib = Matrix3d::Zero( 3, 3 );

		double weight{ 0.0 };
		double buoyancy{ 0.0 };
	};
	struct ThrusterPositions
	{
		MatrixXd T  = MatrixXd::Zero( 6, 5 );
		VectorXd t1 = VectorXd::Zero( 6, 1 );
		VectorXd t2 = VectorXd::Zero( 6, 1 );
		VectorXd t3 = VectorXd::Zero( 6, 1 );
		VectorXd t4 = VectorXd::Zero( 6, 1 );
		VectorXd t5 = VectorXd::Zero( 6, 1 );
		VectorXd u  = VectorXd::Zero( 5, 1 );
	};

	struct Servo
	{};

	VehiclePhysicalModel( std::fstream& config )
	{
		loadPhysicalParameters( config );
	}

private:
	void loadPhysicalParameters( std::fstream& config );

	// Thrust configuration matrix;

	MatrixXd KAll = MatrixXd::Zero( 5, 5 );

	double alpha01 = 0.0;
	double alpha02 = 0.0;

	// Center of Gravity
	Vector3d rg = Vector3d::Zero( 3 );

	// Weight and buoyancy

	// MRB and Ma
	Matrix< double, 6, 6 > Mrb = Matrix< double, 6, 6 >::Zero( 6, 6 );
	Matrix< double, 6, 6 > Ma  = Matrix< double, 6, 6 >::Zero( 6, 6 );

	// Coeffs. of drag
	double Xu = 0.0, Yv = 0.0, Zw = 0.0, Kp = 0.0, Mq = 0.0, Nr = 0.0;
	double Xuu = 0.0, Yvv = 0.0, Zww = 0.0, Kpp = 0.0, Mqq = 0.0, Nrr = 0.0;
	double Xua = 0.0, Yva = 0.0, Zwa = 0.0, Kpa = 0.0, Mqa = 0.0, Nra = 0.0;
	VectorXd vl  = VectorXd::Zero( 6 );
	VectorXd vnl = VectorXd::Zero( 6 );

	// Diagonal matrices of coeffs
	MatrixXd Dl  = MatrixXd::Zero( 6, 6 );
	MatrixXd Dnl = MatrixXd::Zero( 6, 6 );

	// Rate of angular acceleration of thruster. Used in thrust allocation
	double deltaU = 0.0;

// 	static Matrix3d Smtrx( Vector3d r ); // Function creating a special kind of matrix
// 	void init_geometry();                // Initializing mass, inertia moments, rg
// 	void init_drag();                    // initializing drag matrices
// 	void init_thrust();
// 	VectorXd getRestoringForces( VectorXd currentState ); // Getting restoring forces vector

// public:
// 	// VectorXd states = VectorXd::Zero(12);
// 	Matrix< double, 6, 6 > coriolis_matrix( VectorXd cur_state );
// 	Matrix< double, 12, 12 > A_state_matrix( VectorXd cur_state );
// 	Matrix< double, 12, 6 > B_state_matrix();
// 	void thrust_allocation( VectorXd tau );
// 	VectorXd getThrustSignal() const;
// 	VectorXd getAzimuth() const;
// 	VectorXd getFutureState( VectorXd currentState, Matrix1212 A, Matrix126 B, double deltaT );
// 	MatrixXd getNbar( Matrix1212 A, Matrix126 B, Matrix612 K );
};