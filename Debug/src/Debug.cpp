#include "Debug.h"

#include <bitset>
#include <vector>

#include "ROSEnums.h"

void Debug::processInMainLoop()
{
	if( ticks % debugTickSpan == 0 )
	{
		ros::master::getTopics( listOfTopics );

		displayer.displayDebugInfo( this->countAndCompareNumberOfTopics() );
	}
}

void Debug::subscribeTopics()
{
	displayer.addInfoToDisplay(
	    "Health Info", std::vector< std::string >( AUVROS::NodeNames.begin(), AUVROS::NodeNames.end() ), "init" );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::Health::healthReportSummary,
	                                                             AUVROS::QueueSize::GlobalHealthQueueSize,
	                                                             &Debug::displayNodeHealthStatus,
	                                                             this ) );

	displayer.addInfoToDisplay( "Thrusters arbitrarly",
	                            std::vector< std::string >( labels::thrusters.begin(), labels::thrusters.end() ),
	                            "init" );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::DevPC::arbitrarlySetThrusters,
	                                                             AUVROS::QueueSize::StandardQueueSize,
	                                                             &Debug::displayArbitrarlySetThrustersStatus,
	                                                             this ) );

	displayer.addInfoToDisplay(
	    "Servos arbitrarly", std::vector< std::string >( labels::servos.begin(), labels::servos.end() ), "init" );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::DevPC::arbitrarlySetServos,
	                                                             AUVROS::QueueSize::StandardQueueSize,
	                                                             &Debug::displayArbitrarlySetServosStatus,
	                                                             this ) );
	displayer.addInfoToDisplay(
	    "DVL Dead Reckoning",
	    std::vector< std::string >( labels::DVLDeadReckoning.begin(), labels::DVLDeadReckoning.end() ),
	    "init" );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::HardwareSignals::DVLDeadReckoningData,
	                                                             AUVROS::QueueSize::StandardQueueSize,
	                                                             &Debug::displayDVLDeadReckoningStatus,
	                                                             this ) );

	displayer.addInfoToDisplay( "Thrusters regulator",
	                            std::vector< std::string >( labels::thrusters.begin(), labels::thrusters.end() ),
	                            "init" );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::HardwareSignals::signalToThrusters,
	                                                             AUVROS::QueueSize::StandardQueueSize,
	                                                             &Debug::displaySetThrustersRegulatorStatus,
	                                                             this ) );

	displayer.addInfoToDisplay(
	    "Servos regulator", std::vector< std::string >( labels::servos.begin(), labels::servos.end() ), "init" );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::HardwareSignals::signalToServos,
	                                                             AUVROS::QueueSize::StandardQueueSize,
	                                                             &Debug::displaySetServosRegulatorStatus,
	                                                             this ) );

	displayer.addInfoToDisplay(
	    "Global estimated position",
	    std::vector< std::string >( labels::DVLDeadReckoning.begin(), labels::DVLDeadReckoning.end() ),
	    "init" );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::Positions::globalEstimatedPosition,
	                                                             AUVROS::QueueSize::StandardQueueSize,
	                                                             &Debug::displayEstimatedPositionStatus,
	                                                             this ) );

	displayer.addInfoToDisplay(
	    "State Machine", std::vector< std::string >( labels::states.begin(), labels::states.end() ), "init" );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::States::currentStateName,
	                                                             AUVROS::QueueSize::SmallQueue,
	                                                             &Debug::displayCurrentStateStatus,
	                                                             this ) );
}

void Debug::displayNodeHealthStatus( const AUVROS::MessageTypes::HealthReport& report )
{
	std::vector< DataType > values{};

	std::bitset< AUVROS::NodeIDs::Count > nodesHealthSet( report.data );

	for( int i = 0; i < nodesHealthSet.size(); ++i )
	{
		if( nodesHealthSet[ i ] )
		{
			values.emplace_back( "healthy" );
		}
		else
		{
			values.emplace_back( "down" );
		}
	}
	displayer.setMajorColumnValues( DisplayerDataPositions::Health, values );
}

void Debug::displayArbitrarlySetThrustersStatus( const AUVROS::MessageTypes::ThrustersSignal& message )
{
	std::vector< DataType > values;

	for( auto i = 0u; i < message.data.size(); ++i )
	{
		values.emplace_back( std::to_string( message.data.at( i ) ) );
	}
	displayer.setMajorColumnValues( DisplayerDataPositions::ThrustersArbitrarly, values );
}

void Debug::displayArbitrarlySetServosStatus( const AUVROS::MessageTypes::ServosSignal& message )
{
	std::vector< DataType > values;

	for( auto i = 0u; i < message.data.size(); ++i )
	{
		values.emplace_back( std::to_string( message.data.at( i ) ) );
	}
	displayer.setMajorColumnValues( DisplayerDataPositions::ServosArbitrarly, values );
}

void Debug::displayDVLDeadReckoningStatus( const AUVROS::MessageTypes::DVLDeadReckoning& message )
{
	std::vector< DataType > values;

	values.emplace_back( std::to_string( message.data.at( 0 ) ) );
	values.emplace_back( std::to_string( message.data.at( 1 ) ) );
	values.emplace_back( std::to_string( message.data.at( 2 ) ) );
	values.emplace_back( std::to_string( message.data.at( 3 ) ) );
	values.emplace_back( std::to_string( message.data.at( 4 ) ) );
	values.emplace_back( std::to_string( message.data.at( 5 ) ) );

	displayer.setMajorColumnValues( DisplayerDataPositions::DVLDeadReckoning, values );
}

void Debug::displaySetThrustersRegulatorStatus( const AUVROS::MessageTypes::ThrustersSignal& message )
{
	std::vector< DataType > values;

	for( auto i = 0u; i < message.data.size(); ++i )
	{
		values.emplace_back( std::to_string( message.data.at( i ) ) );
	}
	displayer.setMajorColumnValues( DisplayerDataPositions::ThrustersRegulator, values );
}

void Debug::displaySetServosRegulatorStatus( const AUVROS::MessageTypes::ThrustersSignal& message )
{
	std::vector< DataType > values;

	for( auto i = 0u; i < message.data.size(); ++i )
	{
		values.emplace_back( std::to_string( message.data.at( i ) ) );
	}
	displayer.setMajorColumnValues( DisplayerDataPositions::ServosRegulator, values );
}

void Debug::displayEstimatedPositionStatus( const AUVROS::MessageTypes::Position& message )
{
	std::vector< DataType > values;

	values.emplace_back( std::to_string( message.linear.x ) );
	values.emplace_back( std::to_string( message.linear.y ) );
	values.emplace_back( std::to_string( message.linear.z ) );
	values.emplace_back( std::to_string( message.angular.x ) );
	values.emplace_back( std::to_string( message.angular.y ) );
	values.emplace_back( std::to_string( message.angular.z ) );

	displayer.setMajorColumnValues( DisplayerDataPositions::GlobalEstimatedPosition, values );
}

void Debug::displayCurrentStateStatus( const AUVROS::MessageTypes::StateNames& message )
{
	std::vector< DataType > values;

	values.emplace_back( message.data );

	displayer.setMajorColumnValues( DisplayerDataPositions::State, values );
}

void Debug::advertiseTopics() {}
void Debug::connectServices() {}

bool Debug::countAndCompareNumberOfTopics() const
{
	unsigned countActive{ 0u };
	for( auto& in : listOfTopics )
	{
		if( in.name.find( AUVROS::Folders::MainFolderShort ) != std::string::npos )
		{
			++countActive;
		}
	}
	return ( countActive == ( this->rosSubscribers.size() - this->numberofIgnoredTopics ) );
}
