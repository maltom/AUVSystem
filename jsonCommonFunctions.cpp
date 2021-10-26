#include "jsonCommonFunctions.h"

#include <iostream>
#include <exception>
#include <mutex>

#include <Eigen/Dense>

#include "CommonEnums.h"


namespace jsonFunctions
{
std::mutex busy;
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

	data.maxThrust = thrustersData.get< jsonxx::Number >( "maxThrust" );

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
		data.thrusterConfigurations.push_back( thrusterVec );
	}

	delete desiredConfigFile;
	busy.unlock();
	return data;
}

VehiclePhysicalModel::Servos readServosData( configFiles::fileID configID )
{
	busy.lock();
	ConfigFile* desiredConfigFile = new ConfigFile( configID );

	VehiclePhysicalModel::Servos data;

	auto azimuthalThrusters = desiredConfigFile->parsedFile.get< jsonxx::Object >( "vehicle" )
	                              .get< jsonxx::Object >( "thrusters" )
	                              .get< jsonxx::Array >( "azimuthal" );

	for( auto i = 0u; i < azimuthalThrusters.size(); ++i )
	{
		if( azimuthalThrusters.get< jsonxx::Boolean >( i ) )
		{
			data.servoNumberAngle.emplace_back( i, 0.0 );
		}
	}

	auto servoData = desiredConfigFile->parsedFile.get< jsonxx::Object >( "vehicle" ).get< jsonxx::Object >( "servos" );

	data.servoSpeed = servoData.get< jsonxx::Number >( "angleSpeedPerSecond" );

	delete desiredConfigFile;
	busy.unlock();
	return data;
}

VehiclePhysicalModel::Drag readDragData( configFiles::fileID configID )
{
	busy.lock();
	ConfigFile* desiredConfigFile = new ConfigFile( configID );

	VehiclePhysicalModel::Drag data;

	auto dragData
	    = desiredConfigFile->parsedFile.get< jsonxx::Object >( "vehicle" ).get< jsonxx::Object >( "dragCoefficients" );

	auto linearData    = dragData.get< jsonxx::Object >( "linear" );
	auto quadraticData = dragData.get< jsonxx::Object >( "quadratic" );
	auto addedMassData = dragData.get< jsonxx::Object >( "addedMass" );

	data.linear
	    = VehiclePhysicalModel::Drag::Linear{ static_cast< double >( linearData.get< jsonxx::Number >( "Xu" ) ),
		                                      static_cast< double >( linearData.get< jsonxx::Number >( "Yv" ) ),
		                                      static_cast< double >( linearData.get< jsonxx::Number >( "Zw" ) ),
		                                      static_cast< double >( linearData.get< jsonxx::Number >( "Kp" ) ),
		                                      static_cast< double >( linearData.get< jsonxx::Number >( "Mq" ) ),
		                                      static_cast< double >( linearData.get< jsonxx::Number >( "Nr" ) ) };
	data.quadratic = VehiclePhysicalModel::Drag::Quadratic{
		static_cast< double >( quadraticData.get< jsonxx::Number >( "Xuu" ) ),
		static_cast< double >( quadraticData.get< jsonxx::Number >( "Yvv" ) ),
		static_cast< double >( quadraticData.get< jsonxx::Number >( "Zww" ) ),
		static_cast< double >( quadraticData.get< jsonxx::Number >( "Kpp" ) ),
		static_cast< double >( quadraticData.get< jsonxx::Number >( "Mqq" ) ),
		static_cast< double >( quadraticData.get< jsonxx::Number >( "Nrr" ) )
	};
	data.addedMass = VehiclePhysicalModel::Drag::AddedMass{
		static_cast< double >( addedMassData.get< jsonxx::Number >( "Xua" ) ),
		static_cast< double >( addedMassData.get< jsonxx::Number >( "Yva" ) ),
		static_cast< double >( addedMassData.get< jsonxx::Number >( "Zwa" ) ),
		static_cast< double >( addedMassData.get< jsonxx::Number >( "Kpa" ) ),
		static_cast< double >( addedMassData.get< jsonxx::Number >( "Mqa" ) ),
		static_cast< double >( addedMassData.get< jsonxx::Number >( "Nra" ) )
	};

	delete desiredConfigFile;
	busy.unlock();
	return data;
}

} // namespace vehicle

namespace regulator
{
LQRRegulator readLQRData( configFiles::fileID configID )
{
	busy.lock();
	ConfigFile* desiredConfigFile = new ConfigFile( configID );

	LQRRegulator data;

	auto regulatorData = desiredConfigFile->parsedFile.get< jsonxx::Object >( "regulator" );

	auto QVectorData = regulatorData.get< jsonxx::Array >( "QMatrix" );
	auto RVectorData = regulatorData.get< jsonxx::Array >( "RMatrix" );

	if( QVectorData.size() != LQRRegulator::stateDim )
	{
		throw "Wrong number of parameters in QMatrix!";
	}
	if( RVectorData.size() != LQRRegulator::controlDim )
	{
		throw "Wrong number of parameters in RMatrix!";
	}

	VectorXd Qvec = VectorXd::Zero( LQRRegulator::stateDim );
	VectorXd Rvec = VectorXd::Zero( LQRRegulator::controlDim );

	for( auto i = 0u; i < LQRRegulator::stateDim; ++i )
	{
		Qvec( i ) = static_cast< double >( QVectorData.get< jsonxx::Number >( i ) );
	}

	for( auto i = 0u; i < LQRRegulator::controlDim; ++i )
	{
		Rvec( i ) = static_cast< double >( RVectorData.get< jsonxx::Number >( i ) );
	}

	data.Q.diagonal() << Qvec;
	data.R.diagonal() << Rvec;

	delete desiredConfigFile;
	busy.unlock();
	return data;
}
double readWorkingFrequency( configFiles::fileID configID )
{
	busy.lock();
	ConfigFile* desiredConfigFile = new ConfigFile( configID );

	double data;

	auto regulatorData = desiredConfigFile->parsedFile.get< jsonxx::Object >( "regulator" );

	data = static_cast< double >( regulatorData.get< jsonxx::Number >( "regulatorWorkingFrequency" ) );

	delete desiredConfigFile;
	busy.unlock();
	return data;
}

} // namespace regulator
} // namespace jsonFunctions