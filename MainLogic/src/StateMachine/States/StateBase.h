#pragma once

class StateBase
{
private:
protected:
public:
	StateBase()            = default;
	virtual void process() = 0;
};