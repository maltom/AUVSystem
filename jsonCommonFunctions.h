#pragma once
#define CONFIG_FILE_PATH "../../../auvConfig/auvConfig.json"

#include "jsonxx/jsonxx.h"

#include "CommonEnums.h"

using namespace network;

namespace jsonFunctions
{
namespace ROS
{
double readRosRate( const jsonxx::Object& file );
}
namespace network
{
uint16_t readDevicePortNumber( const jsonxx::Object& file, Device device);

}
} // namespace jsonFunctions