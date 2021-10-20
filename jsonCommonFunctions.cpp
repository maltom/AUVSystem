#include "jsonCommonFunctions.h"

#include <exception>
#include <mutex>

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
} // namespace jsonFunctions