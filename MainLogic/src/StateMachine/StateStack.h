#pragma once

#include <exception>
#include <fstream>
#include <memory>
#include <stack>

#include "States/StateBase.h"
#include "States/StateIdle.h"
#include "States/StateEnums.h"

// functions
std::shared_ptr< StateBase > createState( StateEnums stateNum );

// class StateStack
template< typename T >
class StateStack final
{
public:
	StateStack();

	std::shared_ptr< T > getCurrentState() const;
	void setStateInsteadOfCurrent( StateEnums stateEnum );
	void pushStateOnTop( StateEnums stateEnum );
	void popState();
	void popMultipleStates( unsigned number );
	void clearStateStack();

private:
	std::stack< std::shared_ptr< T > > stateStack;
};

// definitions
template< typename T >
StateStack< T >::StateStack()
{
	pushStateOnTop( StateEnums::StartingState );
}

template< typename T >
std::shared_ptr< T > StateStack< T >::getCurrentState() const
{
	return this->stateStack.top();
}

template< typename T >
void StateStack< T >::setStateInsteadOfCurrent( StateEnums stateEnum )
{
	if( this->stateStack.size() > 0 )
	{
		this->stateStack.pop();
		pushStateOnTop( createState( stateEnum ) );
	}
	else
	{
		throw std::runtime_error( "StateStack is empty!" );
	}
}

template< typename T >
void StateStack< T >::popState()
{
	if( this->stateStack.size() > 1 )
	{
		this->stateStack.pop();
	}
	else
	{
		throw std::runtime_error( "StateStack can't be empty!" );
	}
}

template< typename T >
void StateStack< T >::popMultipleStates( unsigned number )
{
	if( this->stateStack.size() > number )
	{
		for( auto i = 0u; i < number; ++i )
		{
			this->stateStack.pop();
		}
	}
	else
	{
		throw std::runtime_error( "Too few states on stack to pop!" );
	}
}

template< typename T >
void StateStack< T >::pushStateOnTop( StateEnums stateEnum )
{
	this->stateStack.emplace( createState( stateEnum ) );
}