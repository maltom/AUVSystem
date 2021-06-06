#pragma once

#include "StateBase.h"

class SubStateBase : public StateBase
{
private:
	StateBase* parentState;

protected:
public:
	SubStateBase( StateBase* parent = nullptr ) : StateBase(), parentState( parent ) {}
};