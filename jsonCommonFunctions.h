#pragma once

#include <ext/stdio_filebuf.h>
#include <chrono>
#include <fstream>
#include <mutex>
#include <thread>
extern "C"
{
#include <sys/file.h> //flock()
}

#include "external/jsonxx/jsonxx.h"

#include "CommonEnums.h"
#include "ThrusterRegulator/src/LQRRegulator.h"
#include "ThrusterRegulator/src/VehiclePhysicalModel.h"

using namespace network;

class configFile;

namespace jsonFunctions
{
extern std::mutex busy;

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
extern VehiclePhysicalModel::Thrusters readThrustersData( configFiles::fileID configID );
extern VehiclePhysicalModel::Servos readServosData( configFiles::fileID configID );
extern VehiclePhysicalModel::Drag readDragData( configFiles::fileID configID );
} // namespace vehicle

namespace regulator
{
extern LQRRegulator readLQRData( configFiles::fileID configID );
extern double readWorkingFrequency( configFiles::fileID configID );
} // namespace regulator
} // namespace jsonFunctions

class ConfigFile
{
public:
	ConfigFile( configFiles::fileID configID )
	{
		configFileFstream.open( configFiles::filePaths.at( configID ), std::ios::in );

		this->fd = static_cast< __gnu_cxx::stdio_filebuf< char >* const >( configFileFstream.rdbuf() )->fd();
		while( flock( fd, LOCK_EX ) != 0 )
		{
			std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
		}

		if( configFileFstream.is_open() )
		{
			parsedFile.parse( configFileFstream );
		}
	}
	~ConfigFile()
	{
		flock( fd, LOCK_UN );
		configFileFstream.close();
	}

public:
	jsonxx::Object parsedFile;

private:
	std::fstream configFileFstream;
	int fd;
};