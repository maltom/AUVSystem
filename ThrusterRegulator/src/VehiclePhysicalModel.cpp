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
	VectorXd MaDiag = VectorXd::Zero( 6 );
	MaDiag << this->dragParams.addedMass.Xua, this->dragParams.addedMass.Yva, this->dragParams.addedMass.Zwa,
	    this->dragParams.addedMass.Kpa, this->dragParams.addedMass.Mqa, this->dragParams.addedMass.Nra;

	this->dragParams.addedMass.Ma = ( -MaDiag ).asDiagonal();

	// Drag matrices

	VectorXd vl  = VectorXd::Zero( 6 );
	VectorXd vnl = VectorXd::Zero( 6 );

	vl << this->dragParams.linear.Xu, this->dragParams.linear.Yv, this->dragParams.linear.Zw,
	    this->dragParams.linear.Kp, this->dragParams.linear.Mq, this->dragParams.linear.Nr;
	vnl << this->dragParams.quadratic.Xuu, this->dragParams.quadratic.Yvv, this->dragParams.quadratic.Zww,
	    this->dragParams.quadratic.Kpp, this->dragParams.quadratic.Mqq, this->dragParams.quadratic.Nrr;

	this->dragParams.Dl  = ( -vl ).asDiagonal();
	this->dragParams.Dnl = ( -vnl ).asDiagonal();
}

// return matrix of specific shape
Matrix3d Smtrx( const Eigen::Vector3d& r )
{
	Eigen::Matrix3d mtrx;
	mtrx << 0.0, -r( 2 ), r( 1 ), r( 2 ), 0.0, -r( 0 ), -r( 1 ), r( 0 ), 0.0;
	return mtrx;
}