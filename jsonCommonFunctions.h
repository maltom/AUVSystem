#pragma once
#define CONFIG_FILE_PATH "../../../auvConfig/auvConfig.json"

#include <fstream>

#include "jsonxx/jsonxx.h"

#include "CommonEnums.h"

using namespace network;

namespace jsonFunctions
{
namespace ROS
{
double readRosRate( std::fstream& rawFile );
}
namespace network
{
uint16_t readDevicePortNumber( std::fstream& rawFile, Device device);

}
} // namespace jsonFunctions