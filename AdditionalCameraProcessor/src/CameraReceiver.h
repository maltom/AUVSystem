#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "CommonEnums.h"

class CameraReceiver final
{
public:
	CameraReceiver();
cv::Mat image;
	void showImage();
private:
};
