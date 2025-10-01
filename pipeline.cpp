#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>

#include "pipeline.hpp"
#include "frameprocessor.hpp"

#define MATCH_SIZE 40
#define OFFSET 5
#define OFFSET_Y 5
#define DEBUG_TESTING

using namespace std;
using namespace cv;

#define M_PI 3.1415926535897932384626433832795

static double rad2Deg(double rad){return rad*(180/M_PI);}//Convert radians to degrees
static double deg2Rad(double deg){return deg*(M_PI/180);}//Convert degrees to radians


Pipeline::Pipeline(float threshold, int octaves)
	: m_frameProcessor("BRISK", threshold, octaves)
{
}

int Pipeline::ProcessVideo(std::string videoFileName, std::string outFileName, long long to)
{
	Mat first, second;
	FeatureInfo firstInfo, secondInfo;
	VideoCapture cap(videoFileName);
	if (!cap.isOpened()){
		std::cout << "suuuuuuuuuuukaaaaaaa" << std::endl;
		return -1;
	}
		// return -1;
	long long cnt = 0;
	if (to < 0)
	{
		to = std::numeric_limits<long long>::max();
	}
	Size imageSize;
	cv::Rect cropRect;

	if (cap.grab() && cnt<to)
	{
		cap >> second;
		cropRect = Rect(OFFSET_Y, OFFSET, second.cols-2*OFFSET_Y, second.rows-2*OFFSET);
		second = Mat(second, cropRect);
		cv::cvtColor(second,second,cv::COLOR_BGR2GRAY);
		imageSize = second.size();
		secondInfo = m_frameProcessor.GetKeypointData(second);
	}

	while (cap.grab() && cnt < to)
	{
		first = second.clone();
		swap(firstInfo, secondInfo);
		cap >> second;
		if (second.rows == 0 || second.cols == 0){
			continue;
		}
		second = Mat(second, cropRect);
		cv::cvtColor(second,second,cv::COLOR_BGR2GRAY);
		cnt++;
		auto result = m_frameProcessor.MatchImages(first, firstInfo, second, secondInfo);
		m_matchedData.push_back(result);
	}

	cap.release();

	cout << "Estimating movems..." << endl;
	for (auto const & item : m_matchedData)
	{
		m_estimator.EstimateMovements(item);
	}
	auto movems = m_estimator.GetMovements();

	std::ofstream out_matrix_with_offset;
	out_matrix_with_offset.open(outFileName);
	for (auto move : movems){
		out_matrix_with_offset << "x shift: " << move.at<double>(0,2) << ", y shift: " << move.at<double>(1,2) <<std::endl;
	}
	out_matrix_with_offset.close();
	return 0;
}