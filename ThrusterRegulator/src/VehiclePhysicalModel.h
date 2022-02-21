#pragma once

#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "CommonEnums.h"

using namespace Eigen;
using namespace regulator;

using TorquesFunctions
    = std::vector< std::tuple< std::function< double() >, std::function< double( double ) >, double > >;

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
		Matrix< double, sixDim, sixDim > Mrb = Matrix< double, sixDim, sixDim >::Zero();

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
			Matrix< double, sixDim, sixDim > Ma = Matrix< double, sixDim, sixDim >::Zero();
		};
		AddedMass addedMass;
		Linear linear;
		Quadratic quadratic;

		Matrix< double, sixDim, sixDim > Dl  = Matrix< double, sixDim, sixDim >::Zero();
		Matrix< double, sixDim, sixDim > Dnl = Matrix< double, sixDim, sixDim >::Zero();
	};

	struct Thrusters
	{
		struct LinearForcesFunction
		{
			double minusFactor{ 1.0 };
			std::function< double( double ) > trigonometricFunction;
		};

		struct TorqueFunction
		{
			double minusFactor{ 1.0 };
			std::function< double( double ) > trigonometricFunction;
			dimensionsIndex dimensionToMultiply{ dimensionsIndex::x };
		};

		struct AzimuthalThrusterFunctions
		{
			std::array< LinearForcesFunction, 3 > linearForces;
			std::array< std::pair< TorqueFunction, TorqueFunction >, 3 > torques;
		};

		// configuration - the  , {x, y, z, p, q, r}
		MatrixXd AllThrustersConfigurationsMatrix = MatrixXd::Zero( sixDim, 5 );

		// VectorXd::Zero( 6, 1 ) x thrusterAmount;
		std::vector< VectorXd > positionsAndRotations;
		// Influence is calculated from position and rotations

		// MatrixXd azimuthalThrustersConfigMatrix;

		std::vector< AzimuthalThrusterFunctions > azimuthalBaseFunctions;
		std::vector< AzimuthalThrusterFunctions > azimuthalDerivativeFunctions;

		// inertia of thruster - how fast can thrusters change their generated thrust per deltaT
		double deltaU{ 0.0 };
		double maxThrust{ 0.0 };
		unsigned numberOfAzimuthalThrusters{ 0u };
		unsigned thrustersAmount{ 0u };
	};

	struct Servos
	{
		std::vector< std::pair< int, std::vector< dimensionsIndex > > > azimuthalThrusterDimensionsOfInfluence;
		double servoSpeed{ 0.0 };
		std::vector< std::pair< double, dimensionsIndex > > servosAngles;
	};

	VehiclePhysicalModel( configFiles::fileID configID )
	{
		loadPhysicalParameters( configID );
		initMatrices();
	}

	Matrix< double, sixDim, sixDim > calculateCoriolisMatrix( const VectorXd& currentState ) const;

	MatrixXd getAzimuthalThrustersConfig() const;
	void calculateAllThrusterConfigutationMatrix();
	void updateAzimuthalThrusterConfig( const std::vector< double >& newServosAngles );

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
	void adjustParametersForWorkingFrequency( const float freq );

	VectorXd getRestoringForces( const VectorXd& currentState ) const; // Getting restoring forces vector

	void initMatrices();

	Inertial inertialParams;
	Thrusters thrusterParams;
	Drag dragParams;
	Servos servos;

	// Thrust configuration matrix;

	MatrixXd KAll = MatrixXd::Zero( 5, 5 );

	// public:
	// 	VectorXd getThrustSignal() const;
	// 	VectorXd getAzimuth() const;
	// 	VectorXd getFutureState( VectorXd currentState, Matrix1212 A, Matrix126 B, double deltaT );
};
