#include "jsonCommonFunctions.h"

#include <exception>
#include <mutex>

#include "CommonEnums.h"

namespace jsonFunctions
{
	
inline std::mutex busy;
namespace ROS
{

double readRosRate( std::fstream& rawFile )
{
	std::fstream config;
	busy.lock();

	config.open(CONFIG_FILE_PATH, std::ios::in);
	jsonxx::Object file;
	file.parse( config );
	auto rosConfig = file.get< jsonxx::Object >( "ROS" );

	busy.unlock();
	return rosConfig.get< jsonxx::Number >( "rate" );
}
} // namespace ROS
namespace network
{
uint16_t readDevicePortNumber( std::fstream& rawFile, Device device )
{
	std::fstream config;
	busy.lock();
	jsonxx::Object file;
	file.parse( config );
	jsonxx::Object networkConfig;
	if( file.has< jsonxx::Object >( "network" ) )
	{
		networkConfig = file.get< jsonxx::Object >( "network" );
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
	busy.unlock();
	return static_cast< uint16_t >( 50000 );
}
} // namespace network
} // namespace jsonFunctions