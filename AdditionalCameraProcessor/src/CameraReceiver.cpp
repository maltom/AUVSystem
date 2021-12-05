#include "CameraReceiver.h"

#include <algorithm>
#include <iostream>
#include <exception>

CameraReceiver::CameraReceiver()
{

	image = cv::imread( "../../ziomki.JPG" );
	cv::namedWindow( "Display", cv::WindowFlags::WINDOW_FULLSCREEN );
}

void CameraReceiver::showImage()
{

	if( !image.data )
	{
		printf( "No image data \n" );
	}
	else
		printf( "udao sie" );
	cv::imshow( "Display", image );
	cv::waitKey( 0 );
}