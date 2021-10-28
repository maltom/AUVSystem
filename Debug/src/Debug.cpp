#include "Debug.h"
#include "ROSEnums.h"
#include <iostream>

void Debug::processInMainLoop()
{
	ros::master::getTopics( listOfTopics );
	if( ticks % debugTickSpan == 0 )
		this->displayDebugInfo();
}

void Debug::subscribeTopics()
{
	
}
void Debug::advertiseTopics() {}
void Debug::connectServices() {}
