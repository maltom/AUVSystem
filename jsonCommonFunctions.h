#pragma once

#include <fstream>

#include "external/jsonxx/jsonxx.h"

#include "CommonEnums.h"

using namespace network;

class configFile;

namespace jsonFunctions
{
extern jsonxx::Object mainConfigFile;

namespace ROS
{
extern double readRosRate( std::fstream& rawFile );
}
namespace network
{
extern uint16_t readDevicePortNumber( std::fstream& rawFile, Device device );

}
} // namespace jsonFunctions

class ConfigFile
{
public:
	ConfigFile( configFiles::fileID )
    {
        configF
    }

	std::fstream configFile;
};