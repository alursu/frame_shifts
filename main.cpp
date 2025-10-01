#include <string>

#include <opencv2/opencv.hpp>

#include "pipeline.hpp"

#include <opencv2/core/utils/logger.hpp>
#include <opencv2/core/utility.hpp>

const cv::String keys =
 "{help h usage ? | | print this message }"
 "{@videoSrc | | video to stitch }"
 "{@outFile | anglechange_without_invert| stitched image }"
 "{N count |-1 | count of frames to stitch }"
 "{t threshold |30 | threshold for stitcher }"
;

int main(int argc, char* argv[])
{
	// Then the logging level can be set with the following function
	cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_ERROR);
	int maxFrames = -1;
	cv::CommandLineParser parser(argc, argv, keys);
	
	std::string videoName = parser.get<std::string>(0);

	if (videoName.empty())
	{
		videoName = "./globus_avi/angle_zoom.avi";
	}
	auto outName = parser.get<std::string>(1) + ".txt";
	std::cout << "Will write result in " << outName << std::endl;

	maxFrames = parser.get<int>("N");

	float threshold = parser.get<float>("t");
	std::cout << "Threshold " << threshold << std::endl;

	Pipeline stitch (threshold);
	stitch.setOutput(outName);

	stitch.ProcessVideo(videoName, outName, maxFrames);
 	return 0;
}