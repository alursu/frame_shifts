#include "pipeline.hpp"

#define MATCH_SIZE 40
#define OFFSET 5
#define OFFSET_Y 5
#define DEBUG_TESTING

using namespace std;
using namespace cv;

#define M_PI 3.1415926535897932384626433832795

Pipeline::Pipeline(int threshold, int octaves)
	: m_frameProcessor("BRISK", threshold, octaves)
{
}

int Pipeline::ProcessVideo(std::string videoFileName, std::string outFileName)
{
	Mat first, second;
	FeatureInfo firstInfo, secondInfo;
	VideoCapture cap(videoFileName);
	if (!cap.isOpened()){ 
		std::cout << "video is not opened" << std::endl;
		return -1;
	}
	Size imageSize;
	cv::Rect cropRect;

	if (cap.grab())
	{
		cap >> second;
		cropRect = Rect(OFFSET_Y, OFFSET, second.cols-2*OFFSET_Y, second.rows-2*OFFSET);
		second = Mat(second, cropRect);
		cv::cvtColor(second,second,cv::COLOR_BGR2GRAY);
		imageSize = second.size();
		secondInfo = m_frameProcessor.GetKeypointData(second);
	}

	std::ofstream shifts;
	shifts.open(outFileName);

	while (cap.grab())
	{
		first = second.clone();
		swap(firstInfo, secondInfo);
		cap >> second;
		if (second.rows == 0 || second.cols == 0){
			continue;
		}
		second = Mat(second, cropRect);
		cv::cvtColor(second,second,cv::COLOR_BGR2GRAY);
		auto result = m_frameProcessor.MatchImages(first, firstInfo, second, secondInfo);
		cv::Mat move = m_estimator.EstimateMovements(result);
		shifts << "x shift: " << move.at<double>(0,2) << ", y shift: " << move.at<double>(1,2) <<std::endl;
	}

	cap.release();
	shifts.close();

	return 0;
}