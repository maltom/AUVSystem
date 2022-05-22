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
	this->rosSubscribers.emplace_back( this->rosNode->subscribe(
	    AUVROS::Topics::DevPC::arbitrarlyPopOneState, AUVROS::QueueSize::SmallQueue, &MainLogic::popState, this ) );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::DevPC::arbitrarlyPopToFundamental,
	                                                             AUVROS::QueueSize::SmallQueue,
	                                                             &MainLogic::popToFundamental,
	                                                             this ) );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::DevPC::arbitrarlyPushEmergency,
	                                                             AUVROS::QueueSize::SmallQueue,
	                                                             &MainLogic::pushEmergency,
	                                                             this ) );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe(
	    AUVROS::Topics::DevPC::arbitrarlyPushMission, AUVROS::QueueSize::SmallQueue, &MainLogic::pushMission, this ) );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::DevPC::arbitrarlyPushSpecificState,
	                                                             AUVROS::QueueSize::SmallQueue,
	                                                             &MainLogic::pushSpecificState,
	                                                             this ) );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe(
	    AUVROS::Topics::DevPC::arbitrarlyNextTask, AUVROS::QueueSize::SmallQueue, &MainLogic::nextTask, this ) );
}

void MainLogic::advertiseTopics()
{
	this->rosPublishers.emplace_back(
	    std::make_unique< ros::Publisher >( this->rosNode->advertise< AUVROS::MessageTypes::StateNames >(
	        AUVROS::Topics::States::currentStateName, AUVROS::QueueSize::SmallQueue, true ) ) );
}
void MainLogic::connectServices() {}

void MainLogic::globalEstimatedPositionObtained( const geometry_msgs::Twist& position ) {}

void MainLogic::popState( const AUVROS::MessageTypes::States& state )
{
	this->stateMachine->popState();
}
void MainLogic::popToFundamental( const AUVROS::MessageTypes::States& state )
{
	this->stateMachine->popToFundamental();
}
void MainLogic::nextTask( const AUVROS::MessageTypes::States& state )
{
	this->stateMachine->nextTask();
}
void MainLogic::pushEmergency( const AUVROS::MessageTypes::States& state )
{
	this->stateMachine->pushStateOnTop( StateType::emergency );
}

void MainLogic::pushMission( const AUVROS::MessageTypes::States& state )
{
	this->stateMachine->pushStateOnTop( StateType::mission );
}
void MainLogic::pushSpecificState( const AUVROS::MessageTypes::States& state )
{
	this->stateMachine->pushStateOnTop( static_cast< StateType >( state.data ) );
}