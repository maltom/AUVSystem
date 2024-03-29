#include <memory>

#include <ros/ros.h>

#include "CommonEnums.h"
#include "ROSEnums.h"
#include "TrajectoryPlanner.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, AUVROS::NodeNames.at( AUVROS::TrajectoryPlanner ) );

	std::shared_ptr< ros::NodeHandle > trajectoryPlannerNode = std::make_shared< ros::NodeHandle >();

	TrajectoryPlanner trajectoryPlanner(
	    trajectoryPlannerNode, configFiles::fileID::main_file, AUVROS::NodeIDs::TrajectoryPlanner );
	trajectoryPlanner.startMainLoop();
}