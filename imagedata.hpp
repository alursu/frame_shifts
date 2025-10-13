#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

// Класс для защищенного хранения и передачи информации об изображениях
class ImageData
{
public:
	ImageData(void);
	~ImageData(void);

	void SetKeypoints(std::vector<cv::KeyPoint> keypoints1, std::vector<cv::KeyPoint> keypoints2) {m_KeypointsFirst=keypoints1; m_KeypointsSecond=keypoints2;}
	std::vector<cv::KeyPoint> FirstKeypoints() const {return m_KeypointsFirst;}
	std::vector<cv::KeyPoint> SecondKeypoints() const {return m_KeypointsSecond;}
	std::vector<std::vector<cv::DMatch>> KnnMatches() const {return m_knnMatches;}
	void SetMatches(std::vector<cv::DMatch> matches) {m_Matches=matches;}
	void SetMatches(std::vector<cv::DMatch> matches, std::vector<std::vector<cv::DMatch>> knnMatches) {m_Matches=matches; m_knnMatches = knnMatches;}
	std::vector<cv::DMatch> Matches() const {return m_Matches;}

protected:

	std::vector<cv::KeyPoint> m_KeypointsFirst, m_KeypointsSecond;
	std::vector<cv::DMatch> m_Matches;
	std::vector<std::vector<cv::DMatch>> m_knnMatches;
};

struct FeatureInfo
{
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
};
