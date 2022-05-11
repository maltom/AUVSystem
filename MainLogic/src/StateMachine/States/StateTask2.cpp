#include "StateTask2.h"
#include <iostream>
#include <utility>

StateReturnType StateTask2::process()
{
	return { StateProcessed::pushChild, *this->childStates.begin() };
}
void StateTask2::checkExitCondition() {}
