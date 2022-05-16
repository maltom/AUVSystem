#include "MainLogic.h"
#include "ROSEnums.h"

void MainLogic::processInMainLoop()
{
	auto stateChanged = stateMachine->process();
	if( stateChanged == StackProcessResult::stateChanged )
	{
		AUVROS::MessageTypes::StateNames stateName;
		stateName.data = this->logicData->currentStateName;
		this->rosPublishers.at( PublishersCodes::stateName )->publish( stateName );
	}
}

void MainLogic::subscribeTopics()
{
	// this->globalEstimatedPositionPublisher =  this->rosNode->advertise< std_msgs::Float32 >(
	// TopicsAndServicesNames::Topics::globalEstimatedPosition, 1000 );
	// this->globalEstimatedPositionSubscriber = this->rosNode->subscribe(
	//     AUVROS::Topics::Positions::globalEstimatedPosition, 1000, &MainLogic::globalEstimatedPositionObtained, this
	//     );
}
void MainLogic::advertiseTopics()
{
	this->rosPublishers.emplace_back(
	    std::make_unique< ros::Publisher >( this->rosNode->advertise< AUVROS::MessageTypes::StateNames >(
	        AUVROS::Topics::States::currentStateName, AUVROS::QueueSize::SmallQueue, true ) ) );
}
void MainLogic::connectServices() {}

void MainLogic::globalEstimatedPositionObtained( const geometry_msgs::Twist& position ) {}
