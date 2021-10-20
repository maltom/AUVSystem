#include "MainLogic.h"
#include "ROSEnums.h"

void MainLogic::startMainLoop()
{
	while( ros::ok() )
	{
		ros::spinOnce();
		stateMachine->process();
		this->rosLoopRate->sleep();
	}
}

void MainLogic::subscribeTopics()
{
	// this->globalEstimatedPositionPublisher =  this->rosNode->advertise< std_msgs::Float32 >(
	// TopicsAndServicesNames::Topics::globalEstimatedPosition, 1000 );
	this->globalEstimatedPositionSubscriber
	    = this->rosNode->subscribe( TopicsAndServicesNames::Topics::globalEstimatedPosition,
	                                1000,
	                                &MainLogic::globalEstimatedPositionObtained,
	                                this );
}
void MainLogic::advertiseTopics() const {}
void MainLogic::connectServices() const {}

void MainLogic::globalEstimatedPositionObtained( const geometry_msgs::Twist& position ) {}
