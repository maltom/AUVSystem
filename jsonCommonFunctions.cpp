#include "jsonCommonFunctions.h"

#include <exception>
#include <mutex>

#include <Eigen/Dense>

#include "CommonEnums.h"

namespace jsonFunctions
{

inline std::mutex busy;
namespace ROS
{

double readRosRate( configFiles::fileID configID )
{
	busy.lock();
	ConfigFile* desiredConfigFile = new ConfigFile( configID );

	auto rosConfig = desiredConfigFile->parsedFile.get< jsonxx::Object >( "ROS" );

	delete desiredConfigFile;
	busy.unlock();
	return rosConfig.get< jsonxx::Number >( "rate" );
}
} // namespace ROS
namespace network
{
uint16_t readDevicePortNumber( configFiles::fileID configID, Device device )
{

	busy.lock();
	ConfigFile* desiredConfigFile = new ConfigFile( configID );

	jsonxx::Object networkConfig;
	if( desiredConfigFile->parsedFile.has< jsonxx::Object >( "network" ) )
	{
		networkConfig = desiredConfigFile->parsedFile.get< jsonxx::Object >( "network" );
	}
	else
	{
		throw "Config file doesn't have any network data!";
	}
	auto devicesConfig = networkConfig.get< jsonxx::Object >( "devices" );
	jsonxx::Object deviceObj;
	switch( device )
	{
	case Device::devPC:
		if( devicesConfig.has< jsonxx::Object >( "devPC" ) )
		{
			deviceObj = devicesConfig.get< jsonxx::Object >( "devPC" );
		}
		else
		{
			throw "Config file doesn't have devPC network data!";
		}
		break;
	case Device::DVL:
		if( devicesConfig.has< jsonxx::Object >( "DVL" ) )
		{
			deviceObj = devicesConfig.get< jsonxx::Object >( "DVL" );
		}
		else
		{
			throw "Config file doesn't have DVL network data!";
		}
		break;
	case Device::jetson:
		if( devicesConfig.has< jsonxx::Object >( "jetson" ) )
		{
			deviceObj = devicesConfig.get< jsonxx::Object >( "jetson" );
		}
		else
		{
			throw "Config file doesn't have jetson network data!";
		}
		break;
	case Device::microcontroller:
		if( devicesConfig.has< jsonxx::Object >( "microcontroller" ) )
		{
			deviceObj = devicesConfig.get< jsonxx::Object >( "microcontroller" );
		}
		else
		{
			throw "Config file doesn't have microcontroller network data!";
		}
		break;
	default:
		break;
	}
	auto result = deviceObj.get< jsonxx::Number >( "port" );

	delete desiredConfigFile;
	busy.unlock();
	return static_cast< uint16_t >( result );
}
} // namespace network

namespace vehicle
{
VehiclePhysicalModel::Inertial readInertialData( configFiles::fileID configID )
{
	busy.lock();
	ConfigFile* desiredConfigFile = new ConfigFile( configID );

	VehiclePhysicalModel::Inertial data;

	auto inertialData
	    = desiredConfigFile->parsedFile.get< jsonxx::Object >( "vehicle" ).get< jsonxx::Object >( "inertial" );

	data.buoyancy = inertialData.get< jsonxx::Number >( "buoyancy" );
	data.mass     = inertialData.get< jsonxx::Number >( "mass" );
	data.weight   = data.mass * 9.8066;

	auto moments = inertialData.get< jsonxx::Object >( "moments" );

	data.Ix  = moments.get< jsonxx::Number >( "Ix" );
	data.Iy  = moments.get< jsonxx::Number >( "Iy" );
	data.Iz  = moments.get< jsonxx::Number >( "Iz" );
	data.Ixy = moments.get< jsonxx::Number >( "Ixy" );
	data.Ixz = moments.get< jsonxx::Number >( "Ixz" );
	data.Iyx = moments.get< jsonxx::Number >( "Iyx" );
	data.Iyz = moments.get< jsonxx::Number >( "Iyz" );
	data.Izx = moments.get< jsonxx::Number >( "Izx" );
	data.Izy = moments.get< jsonxx::Number >( "Izy" );

	data.Ib << data.Ix, -data.Ixy, -data.Ixz, -data.Iyx, data.Iy, -data.Iyz, -data.Izx, -data.Izy, data.Iz;

	auto centerOfBuoyancy = inertialData.get< jsonxx::Array >( "centerOfBuoyancy" );
	data.centerOfBuoyancy << centerOfBuoyancy.get< jsonxx::Number >( 0 ), centerOfBuoyancy.get< jsonxx::Number >( 1 ),
	    centerOfBuoyancy.get< jsonxx::Number >( 2 );

	auto centerOfGravity = inertialData.get< jsonxx::Array >( "centerOfGravity" );
	data.centerOfGravity << centerOfGravity.get< jsonxx::Number >( 0 ), centerOfGravity.get< jsonxx::Number >( 1 ),
	    centerOfGravity.get< jsonxx::Number >( 2 );

	delete desiredConfigFile;
	busy.unlock();
	return data;
}
VehiclePhysicalModel::Thrusters readThrustersData( configFiles::fileID configID )
{
	busy.lock();
	ConfigFile* desiredConfigFile = new ConfigFile( configID );

	VehiclePhysicalModel::Thrusters data;

	auto thrustersData
	    = desiredConfigFile->parsedFile.get< jsonxx::Object >( "vehicle" ).get< jsonxx::Object >( "thrusters" );

	auto thrustersPosRot = thrustersData.get< jsonxx::Array >( "positionsAndRotations" );

	data.maxThrust       = thrustersData.get< jsonxx::Number >( "maxThrust" );
	
	int thrustersAmount  = thrustersPosRot.size();
	data.thrustersAmount = thrustersAmount;

	for( auto i = 0u; i < thrustersAmount; ++i )
	{
		auto oneThrusterPosRot = thrustersPosRot.get< jsonxx::Array >( i );
		VectorXd thrusterVec   = VectorXd::Zero( 6 );
		thrusterVec << static_cast< double >( oneThrusterPosRot.get< jsonxx::Number >( 0 ) ),
		    static_cast< double >( oneThrusterPosRot.get< jsonxx::Number >( 1 ) ),
		    static_cast< double >( oneThrusterPosRot.get< jsonxx::Number >( 2 ) ),
		    static_cast< double >( oneThrusterPosRot.get< jsonxx::Number >( 3 ) ),
		    static_cast< double >( oneThrusterPosRot.get< jsonxx::Number >( 4 ) ),
		    static_cast< double >( oneThrusterPosRot.get< jsonxx::Number >( 5 ) );
		data.thrusterPositions.push_back( thrusterVec );
	}

	delete desiredConfigFile;
	busy.unlock();
	return data;
}

} // namespace vehicle
} // namespace jsonFunctions