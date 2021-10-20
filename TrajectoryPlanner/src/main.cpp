#include <memory>

#include <ros/ros.h>

#include "CommonEnums.h"
#include "TrajectoryPlanner.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, "TrajectoryPlanner" );

	std::shared_ptr< ros::NodeHandle > trajectoryPlannerNode = std::make_shared< ros::NodeHandle >();

	TrajectoryPlanner trajectoryPlanner( trajectoryPlannerNode, configFiles::fileID::main_file );
    trajectoryPlanner.startMainLoop();
}