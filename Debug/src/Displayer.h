#pragma once

#include <any>
#include <string>
#include <utility>
#include <vector>

#include "CommonEnums.h"

using DataType = std::string;

namespace display
{
constexpr unsigned visibleDisplayWidth = 80;
// include new line at the end
constexpr unsigned labelDisplayWidth               = 20u;
constexpr unsigned dataDisplayWidth                = 12u;
constexpr unsigned totalNumberOfMajorColumnsPerRow = 5u;
constexpr unsigned totalDisplayHeight              = 24u;
constexpr unsigned columnSeparatorWidth            = 1u;
constexpr unsigned newLineSignWidth                = 1u;
constexpr unsigned headerDisplayWidth              = labelDisplayWidth + columnSeparatorWidth + dataDisplayWidth;
constexpr unsigned totalDisplayWidth               = columnSeparatorWidth
    + ( labelDisplayWidth + columnSeparatorWidth + dataDisplayWidth + columnSeparatorWidth )
        * totalNumberOfMajorColumnsPerRow
    + newLineSignWidth;
constexpr unsigned rowSeparatorWidth            = totalDisplayWidth - 2 * columnSeparatorWidth;
constexpr std::string_view sideBorderSeparator  = "┃";
constexpr std::string_view majorColumnSeparator = "│";
constexpr std::string_view minorColumnSeparator = "┊";
constexpr std::string_view rowMajorSeparator    = "━";
constexpr std::string_view rowMinorSeparator    = "─";
constexpr std::string_view topLeftCorner        = "┏";
constexpr std::string_view topRightCorner       = "┓";
constexpr std::string_view bottomLeftCorner     = "┗";
constexpr std::string_view bottomRightCorner    = "┛";
constexpr std::string_view leftBorderSeparator  = "┣";
constexpr std::string_view rightBorderSeparator = "┫";
enum class rowType
{
	topBorder,
	majorSeparator,
	minorSeparator,
	header,
	data,
	bottomBorder
};
} // namespace display

class Displayer final
{

public:
	struct TopicDataColumn
	{
		std::string header;
		std::vector< std::pair< std::string, DataType > > dataLabelsFields;
		explicit TopicDataColumn( std::string head, const std::vector< std::string >& labels, DataType init )
		    : header( head )
		{
			this->dataLabelsFields.resize( labels.size() );
			for( auto i = 0u; i < labels.size(); ++i )
			{
				dataLabelsFields.at( i ).first  = labels.at( i );
				dataLabelsFields.at( i ).second = init;
			}
		}
	};

	Displayer()
	{
		setupDisplayPositions();
		setupDisplayLabels();
	}

	void addInfoToDisplay( const std::string& header, const std::vector< std::string >& labels, const DataType& init );
	void setMajorColumnValues( const unsigned dataColumnIndex, const std::vector< DataType >& dataForDisplayer );

	void fillMajorDataRow( std::string& display );
	void displayDebugInfo( bool isNumberOfSubscribedTopicsGood ) const;

private:
	std::string createUtilityRow( const display::rowType type ) const;
	std::string createMajorRow( const unsigned dataRowNumber = 0u ) const;

	void setupDisplayPositions();
	void setupDisplayLabels();

	std::vector< TopicDataColumn > dataColumns;
};
