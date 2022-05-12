#pragma once

#include "CommonEnums.h"
#include "StateBase.h"
#include "StateEnums.h"

#include "jsonCommonFunctions.h"

class StateFindGate : public StateBase
{
private:
protected:
public:
	explicit StateFindGate( LogicCommonData* const data, AdditionalDataParameter param, StateBase* const parentState )
	    : StateBase( parentState, data, StateLevel::final, StateType::findGate, param )
	{
	}
	StateReturnType process() override;
	void checkExitCondition() override;
};