#include "Debug.h"
#include "ROSEnums.h"
#include <iostream>

void Debug::processInMainLoop()
{
	if( ticks % debugTickSpan == 0 )
	{
		ros::master::getTopics( listOfTopics );

		this->displayDebugInfo( this->countAndCompareNumberOfTopics() );
	}
}

void Debug::subscribeTopics() {}
void Debug::advertiseTopics() {}
void Debug::connectServices() {}

bool Debug::countAndCompareNumberOfTopics() const
{
	unsigned countActive;
	for( auto& in : listOfTopics )
	{
		if( in.name.find( AUVROS::Folders::MainFolderShort ) != std::string::npos )
		{
			++countActive;
		}
	}
	return ( countActive == this->rosSubscribers.size() );
}

std::string Debug::createRow( display::rowType type ) const
{
	std::string row;
	// +1 for \n
	row.reserve( display::totalDisplayWidth + 1 );
	switch( type )
	{
	case display::rowType::topBorder:
		row = display::topLeftCorner;
		for( auto i = 0u; i < display::rowSeparatorWidth; ++i )
		{
			row += display::rowSeparator;
		}
		row += display::topRightCorner;
		break;
	case display::rowType::bottomBorder:
		row = display::bottomLeftCorner;
		for( auto i = 0u; i < display::rowSeparatorWidth; ++i )
		{
			row += display::rowSeparator;
		}
		row += display::bottomRightCorner;
		break;
	case display::rowType::separator:
		row = display::leftBorderSeparator;
		for( auto i = 0u; i < display::rowSeparatorWidth; ++i )
		{
			row += display::rowSeparator;
		}
		row += display::rightBorderSeparator;
		break;
	default:
		break;
	}
	row += '\n';
	return row;
}

void Debug::displayDebugInfo( bool isNumberOfSubscribedTopicsGood ) const
{
	std::string finalDisplay;
	finalDisplay.reserve( display::totalDisplayWidth * display::totalDisplayHeight );
	if( !isNumberOfSubscribedTopicsGood )
	{
		finalDisplay = "[WARNING] Number of subscribed topics is different than number of topics in system.\n";
	}
	// top border
	finalDisplay.append( createRow( display::rowType::topBorder ) );

	// add some stuff
	finalDisplay.append( createRow( display::rowType::separator ) );
	finalDisplay.append( createRow( display::rowType::separator ) );

	// bottom
	finalDisplay.append( createRow( display::rowType::bottomBorder ) );
	system( "clear" );
	finalDisplay.shrink_to_fit();
	std::cout << finalDisplay;
}