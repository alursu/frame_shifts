#include <string>

#include <opencv2/opencv.hpp>

#include "pipeline.hpp"

int main(int argc, char* argv[])
{
	std::string videoName = "./globus_avi/anglechange.avi";
	auto outName = "out.txt";
	std::cout << "Will write result in " << outName << std::endl;

	int threshold = 30;
	std::cout << "Threshold " << threshold << std::endl;

	Pipeline stitch (threshold);

	stitch.ProcessVideo(videoName, outName);
 	return 0;
}