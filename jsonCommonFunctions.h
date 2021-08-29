#pragma once
#define CONFIG_FILE_PATH "../../../auvConfig/auvConfig.json"

#include "jsonxx/jsonxx.h"

namespace jsonFunctions
{
double readRosRate( const jsonxx::Object& file );
}