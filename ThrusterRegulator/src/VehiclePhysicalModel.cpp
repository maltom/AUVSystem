#include "VehiclePhysicalModel.h"

#include <iostream>

#include "jsonCommonFunctions.h"

void VehiclePhysicalModel::loadPhysicalParameters( configFiles::fileID configID )
{
	this->inertialParams = jsonFunctions::vehicle::readInertialData( configID );
	this->thrusterParams = jsonFunctions::vehicle::readThrustersData( configID );
	this->thrusterParams.u = VectorXd::Zero( this->thrusterParams.thrustersAmount, 1 );
}