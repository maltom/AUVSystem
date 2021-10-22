#include "Debug.h"
#include "ROSEnums.h"

void Debug::startMainLoop()
{
	while( ros::ok() )
	{
		ros::spinOnce();
		this->rosLoopRate->sleep();
	}
}

void Debug::subscribeTopics()
{
	// this->globalEstimatedPositionPublisher =  this->rosNode->advertise< std_msgs::Float32 >(
	// TopicsAndServicesNames::Topics::globalEstimatedPosition, 1000 );
}
void Debug::advertiseTopics() const {}
void Debug::connectServices() const {}