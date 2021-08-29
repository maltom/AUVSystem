#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>

#include <ros/ros.h>

#include "../jsonCommonFunctions.h"

#include "MainLogic.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, "MainLogic" );

	std::shared_ptr< ros::NodeHandle > mainLogicNodePtr = std::make_shared< ros::NodeHandle >();

	std::fstream configFile;
	configFile.open( CONFIG_FILE_PATH, std::fstream::in );
	if( !configFile.is_open() )
	{
		throw "Can't open config file!";
	}

	MainLogic mainLogic( mainLogicNodePtr, configFile );
    mainLogic.startMainLoop();
}