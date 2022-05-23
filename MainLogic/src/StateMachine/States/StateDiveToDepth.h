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
	explicit StateDiveToDepth( const std::shared_ptr< LogicCommonData >& data,
	                           AdditionalDataParameter param,
	                           const std::shared_ptr< StateBase >& parent )
	    : StateBase( parent, data, StateLevel::final, StateType::diveToDepth, param )
	{
	}
	StateReturnType process() override;
	void checkExitCondition() override;
};