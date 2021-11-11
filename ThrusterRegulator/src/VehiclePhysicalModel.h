#pragma once

#include <fstream>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "CommonEnums.h"

using namespace Eigen;

class VehiclePhysicalModel;

Matrix3d Smtrx( const Eigen::Vector3d& r );

class VehiclePhysicalModel final
{
public:
	struct Inertial
	{
		double mass{ 0.0 };
		double Ix{ 0.0 }, Iy{ 0.0 }, Iz{ 0.0 };
		double Ixy{ 0.0 }, Iyx{ 0.0 }, Ixz{ 0.0 }, Izx{ 0.0 }, Iyz{ 0.0 }, Izy{ 0.0 };
		Matrix3d Ib = Matrix3d::Zero( 3, 3 );

		double weight{ 0.0 };
		double buoyancy{ 0.0 };

		// inertial coefficients matrix
		Matrix< double, 6, 6 > Mrb = Matrix< double, 6, 6 >::Zero( 6, 6 );

		Vector3d centerOfGravity  = Vector3d::Zero();
		Vector3d centerOfBuoyancy = Vector3d::Zero();
	};

	struct Drag
	{
		// Coeffs. of drag
		struct Linear
		{
			double Xu{ 0.0 }, Yv{ 0.0 }, Zw{ 0.0 }, Kp{ 0.0 }, Mq{ 0.0 }, Nr{ 0.0 };
		};
		struct Quadratic
		{
			double Xuu{ 0.0 }, Yvv{ 0.0 }, Zww{ 0.0 }, Kpp{ 0.0 }, Mqq{ 0.0 }, Nrr{ 0.0 };
		};
		struct AddedMass
		{
			double Xua{ 0.0 }, Yva{ 0.0 }, Zwa{ 0.0 }, Kpa{ 0.0 }, Mqa{ 0.0 }, Nra{ 0.0 };
			Matrix< double, 6, 6 > Ma = Matrix< double, 6, 6 >::Zero( 6, 6 );
		};
		AddedMass addedMass;
		Linear linear;
		Quadratic quadratic;

		Matrix< double, 6, 6 > Dl  = Matrix< double, 6, 6 >::Zero();
		Matrix< double, 6, 6 > Dnl = Matrix< double, 6, 6 >::Zero();
	};

	struct Thrusters
	{
		// configuration - the  , {x, y, z, p, q, r}
		MatrixXd AllThrustersConfigurationsMatrix = MatrixXd::Zero( 6, 5 );

		// VectorXd::Zero( 6, 1 ) x thrusterAmount;
		std::vector< VectorXd > thrusterConfigurations;

		// VectorXd::Zero( thrusterAmount, 1 ). u is vector of -1 to 1 values of how each thruster is working
		VectorXd u;

		// inertia of thruster - how fast can thrusters change their generated thrust per deltaT
		double deltaU{ 0.0 };

		double maxThrust;
		int thrustersAmount;
	};

	struct Servos
	{
		// int - thruster number that is attached to servo, double servo Angle
		std::vector< std::pair< int, double > > servoNumberAngle;
		double servoSpeed{ 0.0 };
	};

	VehiclePhysicalModel( configFiles::fileID configID )
	{
		loadPhysicalParameters( configID );
		initMatrices();
	}

	Matrix< double, 6, 6 > calculateCoriolisMatrix( const VectorXd& currentState ) const;

	const Drag& getModelDrag() const
	{
		return this->dragParams;
	}

	const Inertial& getModelInertial() const
	{
		return this->inertialParams;
	}

	const Thrusters& getModelThrusters() const
	{
		return this->thrusterParams;
	}

	const Servos& getModelServos() const
	{
		return this->servos;
	}

private:
	void loadPhysicalParameters( configFiles::fileID configID );
	void adjustParametersForWorkingFrequency( float freq );


	VectorXd getRestoringForces( const VectorXd& currentState ) const; // Getting restoring forces vector
	void allocateThrust( const VectorXd& tau );

	void initMatrices();

	Inertial inertialParams;
	Thrusters thrusterParams;
	Drag dragParams;
	Servos servos;

	// Thrust configuration matrix;

	MatrixXd KAll = MatrixXd::Zero( 5, 5 );

	// public:
	// 	// VectorXd states = VectorXd::Zero(12);
	// 	VectorXd getThrustSignal() const;
	// 	VectorXd getAzimuth() const;
	// 	VectorXd getFutureState( VectorXd currentState, Matrix1212 A, Matrix126 B, double deltaT );
};
