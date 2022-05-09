#pragma once

#include <ext/stdio_filebuf.h>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
extern "C"
{
#include <sys/file.h> //flock()
}
#include <vector>

#include "external/jsonxx/jsonxx.h"

#include "CommonEnums.h"
#include "MainLogic/src/StateMachine/States/StateEnums.h"
#include "ThrusterRegulator/src/LQRRegulator.h"
#include "ThrusterRegulator/src/VehiclePhysicalModel.h"

class configFile;

namespace jsonFunctions
{
extern std::mutex busy;

namespace ROS
{
	extern double readDebugRate( configFiles::fileID configID );
	extern double readRosRate( configFiles::fileID configID );
	extern double readHealthCheckRate( configFiles::fileID configID );
	extern double readHealthReportRate( configFiles::fileID configID );
} // namespace ROS
namespace network
{
	extern uint16_t readDevicePortNumber( configFiles::fileID configID, ::network::Device device );
	extern std::string readDeviceIPNumber( configFiles::fileID configID, ::network::Device device );
} // namespace network

namespace vehicle
{
	extern VehiclePhysicalModel::Inertial readInertialData( configFiles::fileID configID );
	extern VehiclePhysicalModel::Thrusters readThrustersData( configFiles::fileID configID );
	extern VehiclePhysicalModel::Servos readServosData( configFiles::fileID configID );
	extern VehiclePhysicalModel::Drag readDragData( configFiles::fileID configID );
} // namespace vehicle

namespace regulator
{
	extern AllocationPenalizers readPenalizerData( configFiles::fileID configID );
	extern LQRRegulator readLQRData( configFiles::fileID configID );
	extern double readWorkingFrequency( configFiles::fileID configID );
} // namespace regulator

namespace cameras
{
	extern bool checkForStereoscopicCameraPresent( configFiles::fileID configID );
	extern uint8_t countAdditionalCameras( configFiles::fileID configID );
	// extern std::vector< float > getFOV( configFiles::fileID configID );
} // namespace cameras

namespace states
{
	extern std::vector< StateType > readMission( configFiles::fileID configID );
}

} // namespace jsonFunctions

class ConfigFile
{
public:
	explicit ConfigFile( configFiles::fileID configID )
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