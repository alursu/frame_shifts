#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
// #include <opencv2/core/utility.hpp>

#include "pipeline.hpp"

int main(int argc, char* argv[])
{
	cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_ERROR);
	std::string videoName = "test1.avi";
	auto outName = "out.txt";
	std::cout << "Will write result in " << outName << std::endl;

	int threshold = 35;
	std::cout << "Threshold " << threshold << std::endl;

	Pipeline stitch (threshold);

	stitch.ProcessVideo(videoName, outName);
 	return 0;
}