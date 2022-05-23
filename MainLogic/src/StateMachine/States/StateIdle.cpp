#include "StateIdle.h"
#include <iostream>

StateReturnType StateIdle::process()
{
	// std::cout<<"I am Idle ";
	return { StateProcessed::keepCurrentState, StateType::idle };
}

void StateIdle::checkExitCondition() {}
