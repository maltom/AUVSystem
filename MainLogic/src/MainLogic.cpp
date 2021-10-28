#include "MainLogic.h"
#include "ROSEnums.h"

void MainLogic::processInMainLoop()
{
	stateMachine->process();
}

void MainLogic::subscribeTopics()
{
	// this->globalEstimatedPositionPublisher =  this->rosNode->advertise< std_msgs::Float32 >(
	// TopicsAndServicesNames::Topics::globalEstimatedPosition, 1000 );
	this->globalEstimatedPositionSubscriber = this->rosNode->subscribe(
	    AUVROS::Topics::Positions::globalEstimatedPosition, 1000, &MainLogic::globalEstimatedPositionObtained, this );
}
void MainLogic::advertiseTopics() {}
void MainLogic::connectServices() {}

void MainLogic::globalEstimatedPositionObtained( const geometry_msgs::Twist& position ) {}
