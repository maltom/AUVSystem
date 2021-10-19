#include <exception>

#include "jsonCommonFunctions.h"

namespace jsonFunctions
{
namespace ROS
{
double readRosRate( std::fstream& rawFile )
{
	jsonxx::Object file;
	file.parse( rawFile );
	auto rosConfig = file.get< jsonxx::Object >( "ROS" );
	return rosConfig.get< jsonxx::Number >( "rate" );
}
} // namespace ROS
namespace network
{
uint16_t readDevicePortNumber( std::fstream& rawFile, Device device )
{
	jsonxx::Object file;
	file.parse( rawFile );
	auto devicesConfig = file.get< jsonxx::Object >( "network" ).get< jsonxx::Object >( "devices" );
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
	return static_cast< uint16_t >( deviceObj.get< jsonxx::Number >( "port" ) );
}
} // namespace network
} // namespace jsonFunctions