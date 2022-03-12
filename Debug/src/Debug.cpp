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

	displayer.addInfoToDisplay( "DVL Dead Reckoning",
	                            std::vector< std::string >( labels::DVLDeadReckoning.begin(), labels::DVLDeadReckoning.end() ),
	                            "init" );
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::HardwareSignals::DVLDeadReckoningData,
	                                                             AUVROS::QueueSize::StandardQueueSize,
	                                                             &Debug::displayDVLDeadReckoningStatus,
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

void Debug::displayDVLDeadReckoningStatus( const AUVROS::MessageTypes::DVLDeadReckoning& message )
{
	std::vector< DataType > values;
	for( auto i = 0u; i < message.data.size(); ++i )
	{
		values.emplace_back( std::to_string( message.data.at( i ) ) );
	}
	displayer.setMajorColumnValues( DisplayerDataPositions::DVLDeadReckoning, values );
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
