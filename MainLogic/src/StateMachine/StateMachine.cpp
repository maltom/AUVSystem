#include "StateMachine.h"

#include "States/StateBase.h"
#include "StateStack.h"

StateMachine::StateMachine() {}

void StateMachine::process()
{
	auto result = this->stateStack.getCurrentState()->process();

	switch( result )
	{
	case StateReturnType::goToNextState:
		stateStack.popState();

		break;

	default:
		break;
	}
}