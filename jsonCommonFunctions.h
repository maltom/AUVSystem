#pragma once

#include <fstream>

#include "external/jsonxx/jsonxx.h"

#include "CommonEnums.h"
#include "ThrusterRegulator/src/VehiclePhysicalModel.h"

using namespace network;

class configFile;

namespace jsonFunctions
{
namespace ROS
{
extern double readRosRate( configFiles::fileID configID );
}
namespace network
{
extern uint16_t readDevicePortNumber( configFiles::fileID configID, Device device );
}

namespace vehicle
{
    extern VehiclePhysicalModel::Inertial readInertialData( configFiles::fileID configID );
    extern VehiclePhysicalModel::ThrustersData readThrusterPositions( configFiles::fileID configID );
    extern int readThrusterAmount( configFiles::fileID configID );
}
} // namespace jsonFunctions

class ConfigFile
{
public:
	ConfigFile( configFiles::fileID configID )
	{
		configFileFstream.open( configFiles::filePaths.at( configID ) );
		if( configFileFstream.is_open() )
		{
			parsedFile.parse( configFileFstream );
		}
	}
	~ConfigFile()
	{
		configFileFstream.close();
	}
public:
	jsonxx::Object parsedFile;
private:
	std::fstream configFileFstream;
};