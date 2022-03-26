#pragma once

#include <fstream>
#include <memory>

#include "States/StateBase.h"
#include "StateStack.h"

class StateMachine final
{
public:
	StateMachine();
	~StateMachine() {}

	void process();

protected:
private:
	StateStack< StateBase > stateStack;
};