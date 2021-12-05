#include "AdditionalCameraProcessor.h"

#include <bitset>
#include <iostream>
#include <vector>

#include "ROSEnums.h"

void AdditionalCameraProcessor::processInMainLoop()
{

	std::cout << "petla\n";
	receivers.at( 0 ).showImage();
}

void AdditionalCameraProcessor::subscribeTopics() {}

void AdditionalCameraProcessor::advertiseTopics() {}
void AdditionalCameraProcessor::connectServices() {}
