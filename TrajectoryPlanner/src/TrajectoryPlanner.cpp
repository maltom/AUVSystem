#include "TrajectoryPlanner.h"
#include "ROSEnums.h"

void TrajectoryPlanner::startMainLoop()
{
	while( ros::ok() )
	{
		ros::spinOnce();
		rosLoopRate->sleep();
	}
}

void TrajectoryPlanner::subscribeTopics()
{
	// this->globalEstimatedPositionPublisher =  this->rosNode->advertise< std_msgs::Float32 >(
	// TopicsAndServicesNames::Topics::globalEstimatedPosition, 1000 );
}
void TrajectoryPlanner::advertiseTopics() {}
void TrajectoryPlanner::connectServices() {}
