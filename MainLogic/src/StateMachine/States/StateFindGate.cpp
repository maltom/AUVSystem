#include "StateFindGate.h"
#include <iostream>
#include <utility>

StateReturnType StateFindGate::process()
{
	return { StateProcessed::finished, this->getType() };
}
void StateFindGate::checkExitCondition() {}
