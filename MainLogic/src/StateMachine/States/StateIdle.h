#pragma once

#include "StateBase.h"
#include "StateEnums.h"

class StateIdle : public StateBase
{
private:
protected:
public:
	explicit StateIdle( LogicCommonData* const data, AdditionalDataParameter param )
	    : StateBase( nullptr, data, StateLevel::fundamental, StateType::idle, param )
	{
	}
	StateReturnType process() override;
	void checkExitCondition() override;
};