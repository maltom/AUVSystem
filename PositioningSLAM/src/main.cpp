#include <memory>

#include <ros/ros.h>

#include "CommonEnums.h"
#include "PositioningSLAM.h"
#include "ROSEnums.h"

int main( int argc, char** argv )
{

	ros::init( argc, argv, AUVROS::NodeNames.at( AUVROS::PositioningSLAM )  );

	std::shared_ptr< ros::NodeHandle > positioningSLAMNode = std::make_shared< ros::NodeHandle >();

	PositioningSLAM positioningSLAM( positioningSLAMNode, configFiles::fileID::main_file, AUVROS::NodeIDs::PositioningSLAM );
    positioningSLAM.startMainLoop();
}