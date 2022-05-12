#pragma once

#include "CommonEnums.h"
#include "StateBase.h"
#include "StateEnums.h"

#include "jsonCommonFunctions.h"

class StateTask2 : public StateBase
{
private:
protected:
public:
	explicit StateTask2( LogicCommonData* const data, AdditionalDataParameter param, StateBase* const parentState )
	    : StateBase( parentState, data, StateLevel::intermediate, StateType::task2, param )
	{
	}
	void checkExitCondition() override;
};