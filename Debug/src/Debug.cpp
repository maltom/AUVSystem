#include "Debug.h"
#include "ROSEnums.h"
#include <iostream>
void Debug::startMainLoop()
{
	while( ros::ok() )
	{
		ros::spinOnce();
		std::cout<<"git ";
		this->rosLoopRate->sleep();
	}
}

void Debug::subscribeTopics()
{
	// this->globalEstimatedPositionPublisher =  this->rosNode->advertise< std_msgs::Float32 >(
	// TopicsAndServicesNames::Topics::globalEstimatedPosition, 1000 );
}
void Debug::advertiseTopics() {}
void Debug::connectServices() {}
