#pragma once

#include <opencv2/opencv.hpp>
#include <deque>
#include <map>

#include <opencv2/imgproc/imgproc.hpp>

#include "imagedata.hpp"
#include "frameprocessor.hpp"
#include "movesetimator.hpp"

class Pipeline
{
public:
	Pipeline(float threshold = 30, int octaves = 3);

protected:

	FrameProcessor m_frameProcessor;
	MoveEstimator m_estimator;

	std::vector<ImageData> m_matchedData;
	std::string m_outFile;

public:
	int ProcessVideo(std::string videoFileName, std::string outFileName, long long to = -1);
	void setOutput(std::string fName) {m_outFile = fName;};
};

