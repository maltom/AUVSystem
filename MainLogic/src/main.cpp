#include <memory>

#include <ros/ros.h>

#include "CommonEnums.h"
#include "MainLogic.h"
#include "ROSEnums.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, "MainLogic" );

	std::shared_ptr< ros::NodeHandle > mainLogicNode = std::make_shared< ros::NodeHandle >();

	MainLogic mainLogic( mainLogicNode, configFiles::fileID::main_file, AUVROS::NodeIDs::MainLogic );
    mainLogic.startMainLoop();
}