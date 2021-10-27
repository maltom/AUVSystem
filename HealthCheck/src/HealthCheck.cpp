#include "HealthCheck.h"

#define NODE_DUMP_PATH "../../node.dump"

#include <bitset>
#include <ext/stdio_filebuf.h>
extern "C"
{
#include <sys/file.h> //flock()
}

#include "ROSEnums.h"

#include <iostream>

void HealthCheck::processInLoop()
{
	if( ticks % healthCheckTickSpan == 0 )
		this->reportGlobalHealth();
}

void HealthCheck::subscribeTopics()
{
	this->rosSubscribers.emplace_back( this->rosNode->subscribe( AUVROS::Topics::Health::healthReportSingleNode,
	                                                             AUVROS::QueueSize::GlobalHealthQueueSize,
	                                                             &HealthCheck::registerNodeHealthStatus,
	                                                             this ) );
}

void HealthCheck::advertiseTopics()
{
	this->rosPublishers.emplace_back(
	    std::make_unique< ros::Publisher >( this->rosNode->advertise< AUVROS::MessageTypes::HealthReport >(
	        AUVROS::Topics::Health::healthReportSummary, AUVROS::QueueSize::HealthQueueSize ) ) );
}
void HealthCheck::connectServices() {}

void HealthCheck::reportHealth() {}

void HealthCheck::reportGlobalHealth()
{
	this->globalHealthStatus.data |= ( 1 << AUVROS::NodeIDs::HealthCheck );
	this->rosPublishers.at( PublishersCodes::healthSummary )->publish( this->globalHealthStatus );

	std::bitset< AUVROS::NodeIDs::Count > nodesHealthSet( globalHealthStatus.data );
	if( !nodesHealthSet.all() )
	{
		this->writeDeadNodesToDump( nodesHealthSet );
	}

	std::cout << nodesHealthSet << "\n";
	this->globalHealthStatus.data = 0;
}

void HealthCheck::registerNodeHealthStatus( const AUVROS::MessageTypes::HealthReport& report )
{
	this->globalHealthStatus.data |= report.data;
}

void HealthCheck::writeDeadNodesToDump( std::bitset< AUVROS::NodeIDs::Count > nodesHealthSet )
{
	std::fstream dumpFile;

	dumpFile.open( NODE_DUMP_PATH, std::ios::out | std::ios::trunc );

	int fd = static_cast< __gnu_cxx::stdio_filebuf< char >* const >( dumpFile.rdbuf() )->fd();
	while( flock( fd, LOCK_EX ) != 0 )
	{
		std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
	}

	if( dumpFile.is_open() )
	{
		for( auto i = 0u; i < nodesHealthSet.size(); ++i )
		{
			if( nodesHealthSet[ i ] == 0 )
			{
				dumpFile << AUVROS::NodeNames.at( i ) << std::endl;
			}
		}
	}

	dumpFile.close();
	flock( fd, LOCK_UN );
}

void HealthCheck::clearDump()
{
std::fstream dumpFile;

	dumpFile.open( NODE_DUMP_PATH, std::ios::out | std::ios::trunc );
	dumpFile.close();
}