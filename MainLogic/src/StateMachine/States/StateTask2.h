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
	explicit StateTask2( const std::shared_ptr< LogicCommonData >& data,
	                     AdditionalDataParameter param,
	                     const std::shared_ptr< StateBase >& parent )
	    : StateBase( parent, data, StateLevel::intermediate, StateType::task2, param )
	{
	}
	void checkExitCondition() override;
};