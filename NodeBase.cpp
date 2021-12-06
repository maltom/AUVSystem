#include "NodeBase.h"

#include <std_msgs/Int32.h>

#include "jsonCommonFunctions.h"
#include "ROSEnums.h"

void NodeBase::startMainLoop()
{
	while( ros::ok() )
	{
		ros::spinOnce();
		if( ticks % healthReportTickSpan == 0 )
		{
			this->reportHealth();
		}
		++ticks;

		this->processInMainLoop();

		this->rosLoopRate->sleep();
	}
}

void NodeBase::reportHealth()
{
	std_msgs::Int32 healthMessage;
	healthMessage.data = 1 << this->nodeID;
	this->healthReportPublisher.publish( healthMessage );
}