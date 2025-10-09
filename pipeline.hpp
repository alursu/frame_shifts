#pragma once

#include <opencv2/opencv.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <ctime>

#include "imagedata.hpp"
#include "frameprocessor.hpp"
#include "movesetimator.hpp"

class Pipeline
{
public:
	Pipeline(int threshold = 30, int octaves = 3);

protected:

	FrameProcessor m_frameProcessor;
	MoveEstimator m_estimator;

	std::vector<ImageData> m_matchedData;
	std::string m_outFile;

public:
	int ProcessVideo(std::string videoFileName, std::string outFileName);
};

