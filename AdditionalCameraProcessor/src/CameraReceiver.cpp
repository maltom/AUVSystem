#include "CameraReceiver.h"

#include <algorithm>
#include <iostream>
#include <exception>

CameraReceiver::CameraReceiver()
{

	image = cv::imread( "../../ziomki.JPG" );
	cv::namedWindow( "Display Image", cv::WINDOW_AUTOSIZE );
}

void CameraReceiver::showImage()
{
	
	if( !image.data )
	{
		printf( "No image data \n" );
	}
	else
		printf( "udao sie" );
	cv::imshow( "Display Image", image );
}