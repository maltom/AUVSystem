#include "AdditionalCameraProcessor.h"

#include <bitset>
#include <iostream>
#include <vector>

#include "ROSEnums.h"

void AdditionalCameraProcessor::processInMainLoop()
{
	receiver = new CameraReceiver();
	std::cout << "petla\n";
	receiver->showImage();
}

void AdditionalCameraProcessor::subscribeTopics() {}

void AdditionalCameraProcessor::advertiseTopics() {}
void AdditionalCameraProcessor::connectServices() {}
