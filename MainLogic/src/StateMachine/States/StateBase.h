#pragma once

class MainLogic;

class StateBase
{
private:
protected:
	MainLogic* MainLogicParentNode;

public:
	StateBase( MainLogic* logicParentNode = nullptr ) : MainLogicParentNode( logicParentNode ) {}
	virtual void process() = 0;
};