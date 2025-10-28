#pragma once

#include <opencv2/opencv.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <ctime>
#include <cmath>

#include "imagedata.hpp"
#include "frameprocessor.hpp"
#include "movesetimator.hpp"
#include "opticalflowlkt.hpp"
#include "uartmavlink.hpp"

class Pipeline
{
public:
	explicit Pipeline(int threshold = 30, int octaves = 3);

	int ProcessVideo(std::string videoFileName, std::string outFileName);
protected:

	FrameProcessor m_frameProcessor;
	MoveEstimator m_estimator;
	UartMAVlink m_uartMAVlink;

	std::vector<ImageData> m_matchedData;
	std::string m_outFile;

	clock_t previous_img_capture_time = 0;
	//Для siyi-a8 mini
	int camera_hfov = 81;

	float CalculateVerticalFov(float hfov_deg, int width, int height);
};

