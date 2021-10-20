#pragma once

#include <fstream>

#include "external/jsonxx/jsonxx.h"

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