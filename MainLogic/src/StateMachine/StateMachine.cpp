#include "StateMachine.h"
#include "States/StateBase.h"
#include "StateStack.h"

StateMachine::StateMachine()
{
	this->stateStack = std::make_unique< StateStack< StateBase > >();
}

void StateMachine::process()
{
    this->stateStack->getCurrentState()->process();
}