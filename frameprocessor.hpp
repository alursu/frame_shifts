#pragma once

#include <opencv2/features2d.hpp>
#include "imagedata.hpp"
#include <cmath>

class FrameProcessor
{
public:
    FrameProcessor(std::string detectorName, int threshold, int planes);

    FeatureInfo GetKeypointData(cv::Mat const & image);
    ImageData  MatchImages(cv::InputArray& im1, FeatureInfo & first, cv::InputArray& im2,
                                  FeatureInfo & second, double relPower = 0.7);

    int SetThreshold(int newThreshold);

protected:
    ImageData  MatchDescriptors(FeatureInfo const & first, FeatureInfo const & second, double relPower = 0.7);
    bool CheckThreshold(ImageData const & data);

	cv::Ptr<cv::FeatureDetector> m_pDetector;
	cv::Ptr<cv::DescriptorExtractor> m_pDescriptor;
	cv::Ptr<cv::DescriptorMatcher> m_pMatcher;

    double m_minimumFeaturesRequired = 90;
    float constexpr static minimumThreshold = 10.0;
    int m_threshold;
};