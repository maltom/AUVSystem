#include "VehiclePhysicalModel.h"

#include <iostream>

#include "jsonCommonFunctions.h"

void VehiclePhysicalModel::loadPhysicalParameters( configFiles::fileID configID )
{
	this->inertialParams = jsonFunctions::vehicle::readInertialData( configID );
	this->thrusterParams = jsonFunctions::vehicle::readThrustersData( configID );
	this->dragParams     = jsonFunctions::vehicle::readDragData( configID );
	this->servos         = jsonFunctions::vehicle::readServosData( configID );

	this->thrusterParams.u = VectorXd::Zero( this->thrusterParams.thrustersAmount, 1 );
}

void VehiclePhysicalModel::adjustParametersForWorkingFrequency( float freq ) {}

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

	// Thrust Matrix
}

VectorXd VehiclePhysicalModel::getRestoringForces( const VectorXd& currentState ) const
{
	double th = currentState( 4 );
	double ph = currentState( 3 );

	VectorXd restoringForces = VectorXd::Zero( sixDim );
	// tutaj jest mnozenie przez this->inertialParams.centerOfGravity( 2 ) - kiedys to bylo rg(2) - czy nie powinnismy
	// tez gdzies mnozyc tez przez wektor centerOfBuoyancy?
	// z reszta chyba za malo tych mnozen jest
	restoringForces << ( this->inertialParams.weight - this->inertialParams.buoyancy ) * sin( th ),
	    -( this->inertialParams.weight - this->inertialParams.buoyancy ) * cos( th ) * sin( ph ),
	    -( this->inertialParams.weight - this->inertialParams.buoyancy ) * cos( th ) * cos( ph ),
	    this->inertialParams.centerOfGravity( 2 ) * this->inertialParams.weight * cos( th ) * sin( ph ),
	    this->inertialParams.centerOfGravity( 2 ) * this->inertialParams.weight * sin( th ), 0.0;
	return restoringForces;
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
