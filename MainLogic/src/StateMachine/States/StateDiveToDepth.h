#pragma once

#include "CommonEnums.h"
#include "StateBase.h"
#include "StateEnums.h"

#include "jsonCommonFunctions.h"

class StateDiveToDepth : public StateBase
{
private:
protected:
public:
	explicit StateDiveToDepth( LogicCommonData* const data,
	                           AdditionalDataParameter param,
	                           StateBase* const parentState )
	    : StateBase( parentState, data, StateLevel::final, StateType::diveToDepth, param )
	{
	}
	StateReturnType process() override;
	void checkExitCondition() override;
};