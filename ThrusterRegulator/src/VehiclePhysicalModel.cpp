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