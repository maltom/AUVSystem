#include "ThrusterRegulator.h"
#include "ROSEnums.h"

void ThrusterRegulator::startMainLoop()
{
	while( ros::ok() )
	{
		ros::spinOnce();
		rosLoopRate->sleep();
	}
}

void ThrusterRegulator::subscribeTopics()
{
	// this->globalEstimatedPositionPublisher =  this->rosNode->advertise< std_msgs::Float32 >(
	// TopicsAndServicesNames::Topics::globalEstimatedPosition, 1000 );
}
void ThrusterRegulator::advertiseTopics() const {}
void ThrusterRegulator::connectServices() const {}

void ThrusterRegulator::loadRegulatorParameters( configFiles::fileID config )
{
	this->lqrRegulator = jsonFunctions::regulator::readLQRData( config );
}