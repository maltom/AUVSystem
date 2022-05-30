#include "VehiclePhysicalModel.h"

#include <algorithm>
#include <exception>
#include <iostream>

#include "jsonCommonFunctions.h"

void VehiclePhysicalModel::loadPhysicalParameters( configFiles::fileID configID )
{
	this->inertialParams = jsonFunctions::vehicle::readInertialData( configID );
	this->thrusterParams = jsonFunctions::vehicle::readThrustersData( configID );
	this->dragParams     = jsonFunctions::vehicle::readDragData( configID );
	this->servos         = jsonFunctions::vehicle::readServosData( configID );
}

void VehiclePhysicalModel::adjustParametersForWorkingFrequency( const float freq )
{
	this->servos.servoSpeed /= freq;
	this->thrusterParams.deltaU /= freq;
}

void VehiclePhysicalModel::initMatrices()
{
	// Ib matrix
	this->inertialParams.Ib << this->inertialParams.Ix, -this->inertialParams.Ixy, -this->inertialParams.Ixz,
	    -this->inertialParams.Iyx, this->inertialParams.Iy, -this->inertialParams.Iyz, -this->inertialParams.Izx,
	    -this->inertialParams.Izy, this->inertialParams.Iz;

	// Mrb matrix
	this->inertialParams.Mrb.block( 0, 0, 3, 3 ) = this->inertialParams.mass * Matrix3d::Identity( 3, 3 );
	this->inertialParams.Mrb.block( 0, 3, 3, 3 )
	    = -this->inertialParams.mass * Smtrx( this->inertialParams.centerOfGravity );
	this->inertialParams.Mrb.block( 3, 0, 3, 3 )
	    = this->inertialParams.mass * Smtrx( this->inertialParams.centerOfGravity );
	this->inertialParams.Mrb.block( 3, 3, 3, 3 ) = this->inertialParams.Ib;

	// Ma matrix
	VectorXd MaDiag = VectorXd::Zero( sixDim );
	MaDiag << this->dragParams.addedMass.Xua, this->dragParams.addedMass.Yva, this->dragParams.addedMass.Zwa,
	    this->dragParams.addedMass.Kpa, this->dragParams.addedMass.Mqa, this->dragParams.addedMass.Nra;

	this->dragParams.addedMass.Ma = ( -MaDiag ).asDiagonal();

	// Drag matrices

	VectorXd vl  = VectorXd::Zero( sixDim );
	VectorXd vnl = VectorXd::Zero( sixDim );

	vl << this->dragParams.linear.Xu, this->dragParams.linear.Yv, this->dragParams.linear.Zw,
	    this->dragParams.linear.Kp, this->dragParams.linear.Mq, this->dragParams.linear.Nr;
	vnl << this->dragParams.quadratic.Xuu, this->dragParams.quadratic.Yvv, this->dragParams.quadratic.Zww,
	    this->dragParams.quadratic.Kpp, this->dragParams.quadratic.Mqq, this->dragParams.quadratic.Nrr;

	this->dragParams.Dl  = ( -vl ).asDiagonal();
	this->dragParams.Dnl = ( -vnl ).asDiagonal();

	// Thrust matrix
	calculateAllThrusterConfigutationMatrix();

	// simulation max thrust matrix
	const auto& maxThrust = thrusterParams.maxThrust;
	VectorXd KMaxDiag     = VectorXd::Zero( 5 );
	KMaxDiag << maxThrust, maxThrust, maxThrust, maxThrust, maxThrust;
	thrusterParams.KMax = KMaxDiag.asDiagonal();

	// std::cout << thrusterParams.AllThrustersConfigurationsMatrix << std::endl;
	// for( auto i = 0u; i < 5; ++i )
	// {
	// 	std::cout << thrusterParams.AllThrustersConfigurationsMatrix( 5, i ) << " ";
	// }
	// std::cout << std::endl;
}

VectorXd VehiclePhysicalModel::getRestoringForces( const VectorXd& currentState ) const
{
	const auto& theta = currentState( 4 ); // pitch
	const auto& phi   = currentState( 3 ); // roll

	const auto& W = this->inertialParams.weight;
	const auto& B = this->inertialParams.buoyancy;

	const auto& xg = this->inertialParams.centerOfGravity( 0 );
	const auto& yg = this->inertialParams.centerOfGravity( 1 );
	const auto& zg = this->inertialParams.centerOfGravity( 2 );

	const auto& xb = this->inertialParams.centerOfBuoyancy( 0 );
	const auto& yb = this->inertialParams.centerOfBuoyancy( 1 );
	const auto& zb = this->inertialParams.centerOfBuoyancy( 2 );

	VectorXd restoringForces = VectorXd::Zero( sixDim );
	restoringForces << ( W - B ) * sin( theta ), -( W - B ) * cos( theta ) * sin( phi ),
	    -( W - B ) * cos( theta ) * cos( phi ),
	    -( yg * W - yb * B ) * cos( theta ) * cos( phi ) + ( zg * W - zb * B ) * cos( theta ) * sin( phi ),
	    ( zg * W - zb * B ) * sin( theta ) + ( xg * W - xb * B ) * cos( theta ) * cos( phi ),
	    -( xg * W - xb * B ) * cos( theta ) * sin( phi ) - ( yg * W - yb * B ) * sin( theta );

	return restoringForces;
}

MatrixXd VehiclePhysicalModel::getAzimuthalThrustersConfig() const
{
	const auto& startColumn   = this->servos.azimuthalThrusterDimensionsOfInfluence.begin()->first;
	const auto& columnsNumber = this->servos.azimuthalThrusterDimensionsOfInfluence.size();

	MatrixXd azimuthalConfigMatrix
	    = this->thrusterParams.AllThrustersConfigurationsMatrix.block( 0, startColumn, sixDim, columnsNumber );

	return azimuthalConfigMatrix;
}

void VehiclePhysicalModel::calculateAllThrusterConfigutationMatrix()
{

	for( auto i = 0u; i < this->thrusterParams.thrustersAmount; ++i )
	{
		auto additionalPitchForAzimuthal{ 0.0 };
		if( i < this->thrusterParams.numberOfAzimuthalThrusters )
		{
			if( servos.servosAngles.at( i ).second == dimensionsIndex::pitch )
			{
				additionalPitchForAzimuthal = this->servos.servosAngles.at( i ).first;
			}
		}
		const auto& oneThrusterPositionAndRotation = this->thrusterParams.positionsAndRotations.at( i );

		MatrixXd oneThrusterConfig = MatrixXd::Zero( 6, 1 );
		// x, y, z in the robots frame of reference, roll, pitch, yaw calculated from neutral position
		// neutral position is assumed to be in the case of thruster forward thrust vector pointing in x axis
		oneThrusterConfig( dimensionsIndex::x )
		    = std::cos( oneThrusterPositionAndRotation( dimensionsIndex::pitch ) + additionalPitchForAzimuthal )
		    * std::cos( oneThrusterPositionAndRotation( dimensionsIndex::yaw ) );
		oneThrusterConfig( dimensionsIndex::y ) = std::sin( oneThrusterPositionAndRotation( dimensionsIndex::yaw ) )
		    * std::cos( oneThrusterPositionAndRotation( dimensionsIndex::roll ) );
		oneThrusterConfig( dimensionsIndex::z )
		    = -std::sin( oneThrusterPositionAndRotation( dimensionsIndex::pitch ) + additionalPitchForAzimuthal )
		    * std::cos( oneThrusterPositionAndRotation( dimensionsIndex::roll ) );
		// moments
		oneThrusterConfig( dimensionsIndex::roll )
		    = oneThrusterConfig( dimensionsIndex::z ) * oneThrusterPositionAndRotation( dimensionsIndex::y )
		    - oneThrusterConfig( dimensionsIndex::y ) * oneThrusterPositionAndRotation( dimensionsIndex::z );
		oneThrusterConfig( dimensionsIndex::pitch )
		    = oneThrusterConfig( dimensionsIndex::x ) * oneThrusterPositionAndRotation( dimensionsIndex::z )
		    - oneThrusterConfig( dimensionsIndex::z ) * oneThrusterPositionAndRotation( dimensionsIndex::x );
		oneThrusterConfig( dimensionsIndex::yaw )
		    = oneThrusterConfig( dimensionsIndex::y ) * oneThrusterPositionAndRotation( dimensionsIndex::x )
		    - oneThrusterConfig( dimensionsIndex::x ) * oneThrusterPositionAndRotation( dimensionsIndex::y );

		this->thrusterParams.AllThrustersConfigurationsMatrix.block< 6, 1 >( 0, i ) = oneThrusterConfig;

		// std::cout << "Pednik: " << i + 1 << "\n" << oneThrusterConfig << std::endl;
	}
}

void VehiclePhysicalModel::updateAzimuthalThrusterConfig( const std::vector< double >& newServosAngles,
                                                          const VectorXd& thrustSignal_u )
{
	if( newServosAngles.size() != this->thrusterParams.numberOfAzimuthalThrusters )
	{
		throw std::runtime_error( "Wrong number of azimuthal thrusters to update!" );
	}

	for( auto i = 0u; i < newServosAngles.size(); ++i )
	{
		this->servos.servosAngles.at( i ).first = newServosAngles.at( i );
	}

	calculateAllThrusterConfigutationMatrix();
	updateAzimuthalThrustersDifferentialMatrix( thrustSignal_u );
}

void VehiclePhysicalModel::updateAzimuthalThrustersDifferentialMatrix( const VectorXd& thrustSignal_u )
{
	Matrix< double, sixDim, 1 > firstThruster  = Matrix< double, sixDim, 1 >::Zero();
	Matrix< double, sixDim, 1 > secondThruster = Matrix< double, sixDim, 1 >::Zero();

	firstThruster << thrusterParams.azimuthalDerivativeFunctions.at( 0 ).linearForces.at( 0 ).minusFactor
	        * thrusterParams.azimuthalDerivativeFunctions.at( 0 ).linearForces.at( 0 ).trigonometricFunction(
	            this->servos.servosAngles.at( 0 ).first )
	        * thrustSignal_u( 0 ),
	    thrusterParams.azimuthalDerivativeFunctions.at( 0 ).linearForces.at( 1 ).minusFactor
	    * thrusterParams.azimuthalDerivativeFunctions.at( 0 ).linearForces.at( 1 ).trigonometricFunction(
	        this->servos.servosAngles.at( 0 ).first )
	    * thrustSignal_u( 0 ),
	    thrusterParams.azimuthalDerivativeFunctions.at( 0 ).linearForces.at( 2 ).minusFactor
	    * thrusterParams.azimuthalDerivativeFunctions.at( 0 ).linearForces.at( 2 ).trigonometricFunction(
	        this->servos.servosAngles.at( 0 ).first )
	    * thrustSignal_u( 0 ),

	    ( thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 0 ).first.minusFactor
	          * thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 0 ).first.trigonometricFunction(
	              this->servos.servosAngles.at( 0 ).first )
	          * this->thrusterParams.positionsAndRotations.at( 0 )(
	              thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 0 ).first.dimensionToMultiply )
	      + thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 0 ).second.minusFactor
	          * thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 0 ).second.trigonometricFunction(
	              this->servos.servosAngles.at( 0 ).first )
	          * this->thrusterParams.positionsAndRotations.at( 0 )(
	              thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 0 ).second.dimensionToMultiply ) )
	    * thrustSignal_u( 0 ),

	    ( thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 1 ).first.minusFactor
	          * thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 1 ).first.trigonometricFunction(
	              this->servos.servosAngles.at( 0 ).first )
	          * this->thrusterParams.positionsAndRotations.at( 0 )(
	              thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 1 ).first.dimensionToMultiply )
	      + thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 1 ).second.minusFactor
	          * thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 1 ).second.trigonometricFunction(
	              this->servos.servosAngles.at( 0 ).first )
	          * this->thrusterParams.positionsAndRotations.at( 0 )(
	              thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 1 ).second.dimensionToMultiply ) )
	    * thrustSignal_u( 0 ),

	    ( thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 2 ).first.minusFactor
	          * thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 2 ).first.trigonometricFunction(
	              this->servos.servosAngles.at( 0 ).first )
	          * this->thrusterParams.positionsAndRotations.at( 0 )(
	              thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 2 ).first.dimensionToMultiply )
	      + thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 2 ).second.minusFactor
	          * thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 2 ).second.trigonometricFunction(
	              this->servos.servosAngles.at( 0 ).first )
	          * this->thrusterParams.positionsAndRotations.at( 0 )(
	              thrusterParams.azimuthalDerivativeFunctions.at( 0 ).torques.at( 2 ).second.dimensionToMultiply ) )
	    * thrustSignal_u( 0 );

	secondThruster << thrusterParams.azimuthalDerivativeFunctions.at( 1 ).linearForces.at( 0 ).minusFactor
	        * thrusterParams.azimuthalDerivativeFunctions.at( 1 ).linearForces.at( 0 ).trigonometricFunction(
	            this->servos.servosAngles.at( 1 ).first )
	        * thrustSignal_u( 1 ),
	    thrusterParams.azimuthalDerivativeFunctions.at( 1 ).linearForces.at( 1 ).minusFactor
	    * thrusterParams.azimuthalDerivativeFunctions.at( 1 ).linearForces.at( 1 ).trigonometricFunction(
	        this->servos.servosAngles.at( 1 ).first )
	    * thrustSignal_u( 1 ),
	    thrusterParams.azimuthalDerivativeFunctions.at( 1 ).linearForces.at( 2 ).minusFactor
	    * thrusterParams.azimuthalDerivativeFunctions.at( 1 ).linearForces.at( 2 ).trigonometricFunction(
	        this->servos.servosAngles.at( 1 ).first )
	    * thrustSignal_u( 1 ),

	    ( thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 0 ).first.minusFactor
	          * thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 0 ).first.trigonometricFunction(
	              this->servos.servosAngles.at( 1 ).first )
	          * this->thrusterParams.positionsAndRotations.at( 1 )(
	              thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 0 ).first.dimensionToMultiply )
	      + thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 0 ).second.minusFactor
	          * thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 0 ).second.trigonometricFunction(
	              this->servos.servosAngles.at( 1 ).first )
	          * this->thrusterParams.positionsAndRotations.at( 1 )(
	              thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 0 ).second.dimensionToMultiply ) )
	    * thrustSignal_u( 1 ),

	    ( thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 1 ).first.minusFactor
	          * thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 1 ).first.trigonometricFunction(
	              this->servos.servosAngles.at( 1 ).first )
	          * this->thrusterParams.positionsAndRotations.at( 1 )(
	              thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 1 ).first.dimensionToMultiply )
	      + thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 1 ).second.minusFactor
	          * thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 1 ).second.trigonometricFunction(
	              this->servos.servosAngles.at( 1 ).first )
	          * this->thrusterParams.positionsAndRotations.at( 1 )(
	              thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 1 ).second.dimensionToMultiply ) )
	    * thrustSignal_u( 1 ),

	    ( thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 2 ).first.minusFactor
	          * thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 2 ).first.trigonometricFunction(
	              this->servos.servosAngles.at( 1 ).first )
	          * this->thrusterParams.positionsAndRotations.at( 1 )(
	              thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 2 ).first.dimensionToMultiply )
	      + thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 2 ).second.minusFactor
	          * thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 2 ).second.trigonometricFunction(
	              this->servos.servosAngles.at( 1 ).first )
	          * this->thrusterParams.positionsAndRotations.at( 1 )(
	              thrusterParams.azimuthalDerivativeFunctions.at( 1 ).torques.at( 2 ).second.dimensionToMultiply ) )
	    * thrustSignal_u( 1 );

	this->thrusterParams.AzimuthalThrustersDifferentialConfig.block< sixDim, 1 >( 0, 0 ) = firstThruster;
	this->thrusterParams.AzimuthalThrustersDifferentialConfig.block< sixDim, 1 >( 0, 1 ) = secondThruster;
}

Matrix< double, sixDim, sixDim > VehiclePhysicalModel::calculateCoriolisMatrix( const VectorXd& currentState ) const
{
	// Initializing parameters
	Matrix< double, sixDim, sixDim > MrbTemp = Matrix< double, sixDim, sixDim >::Zero();
	Matrix< double, sixDim, sixDim > MaTemp  = Matrix< double, sixDim, sixDim >::Zero();

	Matrix< double, sixDim, sixDim > Crb = Matrix< double, sixDim, sixDim >::Zero();
	Matrix< double, sixDim, sixDim > Ca  = Matrix< double, sixDim, sixDim >::Zero();
	Matrix< double, sixDim, 1 > speed    = Matrix< double, sixDim, 1 >::Zero();

	Matrix3d M11 = Matrix3d::Zero( 3, 3 ), M12 = Matrix3d::Zero( 3, 3 ), M21 = Matrix3d::Zero( 3, 3 ),
	         M22   = Matrix3d::Zero( 3, 3 );
	Matrix3d M11_A = Matrix3d::Zero( 3, 3 ), M12_A = Matrix3d::Zero( 3, 3 ), M21_A = Matrix3d::Zero( 3, 3 ),
	         M22_A = Matrix3d::Zero( 3, 3 );
	Vector3d nu1 = Vector3d::Zero( 3 ), nu2 = Vector3d::Zero( 3 );

	// Splitting linear and angular velocities into 2 vectors
	speed = currentState.block( sixDim, 0, sixDim, 1 );
	nu1   = speed.block( 0, 0, 3, 1 );
	nu2   = speed.block( 3, 0, 3, 1 );

	// Creating sub-matrices derived from rigid-body mass matrix
	MrbTemp = 0.5 * ( this->inertialParams.Mrb + this->inertialParams.Mrb.transpose() ); // Making matrix square
	MaTemp  = 0.5 * ( this->dragParams.addedMass.Ma + this->dragParams.addedMass.Ma.transpose() );

	M11 = MrbTemp.topLeftCorner( 3, 3 );
	M12 = MrbTemp.topRightCorner( 3, 3 );
	M21 = M12.transpose();
	M22 = MrbTemp.bottomRightCorner( 3, 3 );

	M11_A = MaTemp.topLeftCorner( 3, 3 );
	M12_A = MaTemp.topRightCorner( 3, 3 );
	M21_A = M12_A.transpose();
	M22_A = MaTemp.bottomRightCorner( 3, 3 );

	// Creating Coriolis forces matrix
	Crb << Matrix3d::Zero( 3, 3 ), -Smtrx( M11 * nu1 + M12 * nu2 ), -Smtrx( M11 * nu1 + M12 * nu2 ),
	    -Smtrx( M21 * nu1 + M22 * nu2 );

	Ca << Matrix3d::Zero( 3, 3 ), -Smtrx( M11_A * nu1 + M12_A * nu2 ), -Smtrx( M11_A * nu1 + M12_A * nu2 ),
	    -Smtrx( M21_A * nu1 + M22_A * nu2 );

	return Crb + Ca;
}

// return matrix of specific shape
Matrix3d Smtrx( const Eigen::Vector3d& r )
{
	Eigen::Matrix3d mtrx;
	mtrx << 0.0, -r( 2 ), r( 1 ), r( 2 ), 0.0, -r( 0 ), -r( 1 ), r( 0 ), 0.0;
	return mtrx;
}
