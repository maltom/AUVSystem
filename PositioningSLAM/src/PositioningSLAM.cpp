#include "PositioningSLAM.h"
#include "ROSEnums.h"

void PositioningSLAM::startMainLoop()
{
	while( ros::ok() )
	{
		ros::spinOnce();
		rosLoopRate->sleep();
	}
}

void PositioningSLAM::subscribeTopics()
{
	// this->globalEstimatedPositionPublisher =  this->rosNode->advertise< std_msgs::Float32 >(
	// TopicsAndServicesNames::Topics::globalEstimatedPosition, 1000 );
}
void PositioningSLAM::advertiseTopics() {}
void PositioningSLAM::connectServices() {}
