#include <memory>

#include <ros/ros.h>

#include "CommonEnums.h"
#include "ROSEnums.h"
#include "ThrusterRegulator.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, "ThrusterRegulator" );

	std::shared_ptr< ros::NodeHandle > thrusterRegulatorNode = std::make_shared< ros::NodeHandle >();

	ThrusterRegulator thrusterRegulator(
	    thrusterRegulatorNode, configFiles::fileID::main_file, AUVROS::NodeIDs::ThrusterController );
	thrusterRegulator.startMainLoop();
}