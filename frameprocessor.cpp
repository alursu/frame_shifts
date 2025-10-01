#include "frameprocessor.hpp"

using namespace std;
using namespace cv;

FrameProcessor::FrameProcessor(std::string detectorName, float threshold, int planes)
	: m_threshold(threshold)
{
    m_pDetector = BRISK::create(threshold, planes);
    m_pDescriptor = BRISK::create(threshold, planes);
    m_pMatcher = BFMatcher::create(NORM_HAMMING, false);
}

FeatureInfo FrameProcessor::GetKeypointData(cv::Mat const &image)
{
	FeatureInfo frameInfo;
	m_pDetector->detect(image, frameInfo.keypoints);
	m_pDescriptor->compute(image, frameInfo.keypoints, frameInfo.descriptors);
    return frameInfo;
}

ImageData FrameProcessor::MatchImages(cv::InputArray& im1, FeatureInfo & first, cv::InputArray& im2,
                                  FeatureInfo & second, double relPower)
{
	second = GetKeypointData(im2.getMat());
	ImageData ret = MatchDescriptors(first, second, relPower);
	while (!CheckThreshold(ret))
	{
		cout << "New threshold " << m_threshold << endl;
		first = GetKeypointData(im1.getMat());
		second = GetKeypointData(im2.getMat());
		ret = MatchDescriptors(first, second, relPower);
	}
	return ret;
}

float FrameProcessor::SetThreshold(float newThreshold)
{
	cv::Ptr<cv::BRISK> ptr = m_pDetector.dynamicCast<cv::BRISK>();
	ptr->setThreshold(newThreshold);
	return ptr->getThreshold();
}

ImageData FrameProcessor::MatchDescriptors(FeatureInfo const &first, FeatureInfo const &second, double relPower)
{
	vector<cv::DMatch> goodMatches;
	std::vector<std::vector<cv::DMatch>> knnMatches;
	m_pMatcher->knnMatch(first.descriptors, second.descriptors, knnMatches,2);
	for (auto match : knnMatches)
	{
		if (match[0].distance < relPower * match[1].distance)
			goodMatches.push_back(match[0]);
	}
	ImageData retData;
	retData.SetKeypoints(first.keypoints,second.keypoints);
	retData.SetMatches(goodMatches);
	return retData;
}

bool FrameProcessor::CheckThreshold(ImageData const & data)
{
	auto totalMatches = data.Matches().size();
	if (totalMatches < m_minimumFeaturesRequired && m_threshold>minimumThreshold)
	{
		m_threshold = SetThreshold(--m_threshold);
		return false;
	}
	if (totalMatches > 5 * m_minimumFeaturesRequired)
	{
		m_threshold = SetThreshold(++m_threshold);
		return false;
	}
    return true;
}
