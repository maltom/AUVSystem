#include <exception>
#include <fstream>
#include <memory>

#include <ros/ros.h>

#include "ThrusterRegulator.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, "ThrusterRegulator" );

	std::shared_ptr< ros::NodeHandle > thrusterRegulatorNode = std::make_shared< ros::NodeHandle >();

	std::fstream configFile;
	configFile.open( CONFIG_FILE_PATH, std::fstream::in );
	if( !configFile.is_open() )
	{
		throw "Can't open config file!";
	}

	ThrusterRegulator thrusterRegulator( thrusterRegulatorNode, configFile );
    thrusterRegulator.startMainLoop();
	
	configFile.close();
}