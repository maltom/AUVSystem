#include "StateTask1.h"
#include <iostream>
#include <utility>

StateReturnType StateTask1::process()
{
	return { StateProcessed::pushChild, *this->childStates.begin() };
}
void StateTask1::checkExitCondition() {}
