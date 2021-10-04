#pragma once

#include "StateBase.h"
class MainLogic;

class StateIdle : public StateBase
{
private:
protected:
public:
	StateIdle() : StateBase() {}
	void process() override;
};