#include "StateGoLittleTowardsGate.h"
#include <iostream>
#include <utility>

StateReturnType StateGoLittleTowardsGate::process()
{
	return { StateProcessed::keepCurrentState, this->getType() };
}
void StateGoLittleTowardsGate::checkExitCondition() {}
