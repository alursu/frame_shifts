#ifndef OPTICAL_FLOW_H_
#define OPTICAL_FLOW_H_

#include <opencv2/opencv.hpp>
#include <ctime>

class OpticalFlowLkt
{
public:
	cv::Point2f GetOpticalFlow(const cv::Mat& curr_image, bool include_augmented_image = false, bool rev_flow = false);

protected:
	// std::string GetAugmentedImage (std::string& image_base64);

	cv::Mat prev_image;
	double crop_factor = 0.6;

private:
	// Parameters for Shi-Tomasi corner detection
    int maxCorners = 50;
    double qualityLevel = 0.2;
    double minDistance = 10;
    int blockSize = 5;

	// Parameters for Lucas-Kanade optical flow
    cv::Size winSize = cv::Size(15, 15);
    int maxLevel = 2;
    cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::EPS || cv::TermCriteria::COUNT, 20, 0.03);
};

#endif // OPTICAL_FLOW_H_