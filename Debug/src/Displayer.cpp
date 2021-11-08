#include "Displayer.h"

#include <iostream>
#include <exception>

void Displayer::addInfoToDisplay( const std::string& header, const std::vector< std::string >& labels, DataType init )
{
	this->dataColumns.emplace_back( header, labels, init );
}

void Displayer::setMajorColumnValues( unsigned dataColumnIndex, const std::vector< DataType >& dataForDisplayer )
{
	if( this->dataColumns.at( dataColumnIndex ).dataLabelsFields.size() != dataForDisplayer.size() )
	{
		throw "Wrong data or data size for display.";
	}

	for( auto i = 0u; i < dataForDisplayer.size(); ++i )
	{
		this->dataColumns.at( dataColumnIndex ).dataLabelsFields.at( i ).second = dataForDisplayer.at( i );
	}
}

std::string Displayer::createUtilityRow( display::rowType type ) const
{
	std::string row;
	row.reserve( display::totalDisplayWidth );
	switch( type )
	{
	case display::rowType::topBorder:
		row = display::topLeftCorner;
		for( auto i = 0u; i < display::rowSeparatorWidth; ++i )
		{
			row += display::rowMajorSeparator;
		}
		row += display::topRightCorner;
		break;
	case display::rowType::bottomBorder:
		row = display::bottomLeftCorner;
		for( auto i = 0u; i < display::rowSeparatorWidth; ++i )
		{
			row += display::rowMajorSeparator;
		}
		row += display::bottomRightCorner;
		break;
	case display::rowType::majorSeparator:
		row = display::leftBorderSeparator;
		for( auto i = 0u; i < display::rowSeparatorWidth; ++i )
		{
			row += display::rowMajorSeparator;
		}
		row += display::rightBorderSeparator;
		break;
	case display::rowType::minorSeparator:
		row = display::leftBorderSeparator;
		for( auto i = 0u; i < display::rowSeparatorWidth; ++i )
		{
			row += display::rowMinorSeparator;
		}
		row += display::rightBorderSeparator;
		break;
	default:
		break;
	}
	row += '\n';
	return row;
}
std::string Displayer::createMajorRow( unsigned dataRowNumber ) const
{
	std::string dataRow;
	std::string headerRow;
	headerRow.reserve( display::totalDisplayWidth );

	headerRow = display::sideBorderSeparator;
	int bias{ -1 }; // adjusting padding for signs longer than one 1B
	for( auto i = 0u; i < display::totalNumberOfMajorColumns; ++i )
	{
		if( i < dataColumns.size() )
		{
			headerRow += " ";
			headerRow += dataColumns.at( i ).header;

			for( auto j = 0u;
			     j < display::dataDisplayWidth + display::labelDisplayWidth - dataColumns.at( i ).header.size() - 1;
			     ++j )
			{
				headerRow += " ";
			}
			headerRow += display::majorColumnSeparator;
			bias -= 2;
		}
		else
		{
			for( auto j = headerRow.size(); j < display::totalDisplayWidth - bias; ++j )
			{
				headerRow += " ";
			}
			break;
		}
	}
	headerRow += display::sideBorderSeparator;
	headerRow += "\n";

	dataRow = headerRow;
	dataRow.append( createUtilityRow( display::rowType::minorSeparator ) );

	return dataRow;
}

void Displayer::displayDebugInfo( bool isNumberOfSubscribedTopicsGood ) const
{
	std::string finalDisplay;
	finalDisplay.reserve( display::totalDisplayWidth * display::totalDisplayHeight );
	if( !isNumberOfSubscribedTopicsGood )
	{
		finalDisplay = "[WARNING] Number of subscribed topics is different than number of topics in system.\n";
	}

	// top border
	finalDisplay.append( createUtilityRow( display::rowType::topBorder ) );

	// add some stuff
	finalDisplay.append( createMajorRow() );
	finalDisplay.append( createUtilityRow( display::rowType::majorSeparator ) );
	finalDisplay.append( createUtilityRow( display::rowType::majorSeparator ) );

	// bottom
	finalDisplay.append( createUtilityRow( display::rowType::bottomBorder ) );
	system( "clear" );
	finalDisplay.shrink_to_fit();
	std::cout << finalDisplay;
}

void Displayer::setupDisplayPositions() {}
void Displayer::setupDisplayLabels() {}
