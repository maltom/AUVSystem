#include <memory>

#include <ros/ros.h>

#include "AdditionalCameraProcessor.h"
#include "CommonEnums.h"
#include "ROSEnums.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, AUVROS::NodeNames.at( AUVROS::AdditionalCameraProcessor ) );

	std::shared_ptr< ros::NodeHandle > debugNode = std::make_shared< ros::NodeHandle >();

	AdditionalCameraProcessor processor( debugNode, configFiles::fileID::main_file, AUVROS::NodeIDs::AdditionalCameraProcessor );
	processor.startMainLoop();
}