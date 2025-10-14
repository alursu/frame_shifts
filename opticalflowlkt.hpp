#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <ctime>
#include <cmath>
#include <vector>
#include <string>
#include <exception>
#include <numeric>
#include <sstream>
#include <iomanip>
#include <iterator>
#include <algorithm>
#include <stdexcept>

class Opticalflow
{
public:
	Opticalflow();
protected:
	cv::Mat prev_image;
	double prev_image_time = 0.0;
public:

};