#include <exception>
#include <fstream>
#include <iostream>
#include <memory>

#include <ros/ros.h>

#include "../jsonCommonFunctions.h"

#include "TrajectoryPlanner.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, "TrajectoryPlanner" );

	std::shared_ptr< ros::NodeHandle > trajectoryPlannerNode = std::make_shared< ros::NodeHandle >();

	std::fstream configFile;
	configFile.open( CONFIG_FILE_PATH, std::fstream::in );
	if( !configFile.is_open() )
	{
		throw "Can't open config file!";
	}

	TrajectoryPlanner trajectoryPlanner( trajectoryPlannerNode, configFile );
    trajectoryPlanner.startMainLoop();
}