#include "StateDiveToDepth.h"
#include <iostream>
#include <utility>

StateReturnType StateDiveToDepth::process()
{
	return { StateProcessed::finished, this->getType() };
}
void StateDiveToDepth::checkExitCondition() {}
