#include "ThrusterRegulator.h"
#include "jsonCommonFunctions.h"
#include "ROSEnums.h"

void ThrusterRegulator::startMainLoop()
{
	ros::Rate rosRate( jsonFunctions::ROS::readRosRate( configFile ) );
	while( ros::ok() )
	{
		ros::spinOnce();
		rosRate.sleep();
	}
}

void ThrusterRegulator::subscribeTopics()
{
	// this->globalEstimatedPositionPublisher =  this->rosNode->advertise< std_msgs::Float32 >(
	// TopicsAndServicesNames::Topics::globalEstimatedPosition, 1000 );
}
void ThrusterRegulator::advertiseTopics() const {}
void ThrusterRegulator::connectServices() const {}
