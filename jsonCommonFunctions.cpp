#include "jsonCommonFunctions.h"

namespace jsonFunctions
{
double readRosRate( const jsonxx::Object& file )
{
	auto rosConfig = file.get< jsonxx::Object >( "ROS" );
	return rosConfig.get< jsonxx::Number >( "rate" );
}
} // namespace jsonFunctions