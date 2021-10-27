#include "HealthCheck.h"
#include "ROSEnums.h"
#include <iostream>
#include <bitset>

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

void HealthCheck::reportHealth(){}

void HealthCheck::reportGlobalHealth()
{
	this->globalHealthStatus.data |= ( 1 << AUVROS::NodeIDs::HealthCheck );
	this->rosPublishers.at( PublishersCodes::healthSummary )->publish( this->globalHealthStatus );
	std::bitset< 8 > x( globalHealthStatus.data );
	std::cout << x << "\n";
	this->globalHealthStatus.data = 0;
}

void HealthCheck::registerNodeHealthStatus( const AUVROS::MessageTypes::HealthReport& report )
{
	this->globalHealthStatus.data |= report.data;
}
