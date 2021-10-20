#pragma once

#include <fstream>

#include "external/jsonxx/jsonxx.h"

#include "CommonEnums.h"

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