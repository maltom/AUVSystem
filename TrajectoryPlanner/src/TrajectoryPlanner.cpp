#include "TrajectoryPlanner.h"
#include "../jsonCommonFunctions.h"
#include "../ROSEnums.h"

void TrajectoryPlanner::startMainLoop() const
{
	ros::Rate rosRate( jsonFunctions::readRosRate( configFile ) );
	while( ros::ok() )
	{
		ros::spinOnce();
		rosRate.sleep();
	}
}

void TrajectoryPlanner::subscribeTopics()
{
	// this->globalEstimatedPositionPublisher =  this->rosNode->advertise< std_msgs::Float32 >(
	// TopicsAndServicesNames::Topics::globalEstimatedPosition, 1000 );
}
void TrajectoryPlanner::advertiseTopics() const {}
void TrajectoryPlanner::connectServices() const {}
