#ifndef IMAGE_DATA_H_
#define IMAGE_DATA_H_

#include <opencv2/opencv.hpp>
#include <vector>

// Класс для защищенного хранения и передачи информации об изображениях
class ImageData
{
public:

	void set_keypoints(std::vector<cv::KeyPoint> keypoints1, 
					   std::vector<cv::KeyPoint> keypoints2) {keypoints_first_=keypoints1; 
															  keypoints_second_=keypoints2;}
	std::vector<cv::KeyPoint> first_keypoints() const {return keypoints_first_;}
	std::vector<cv::KeyPoint> second_keypoints() const {return keypoints_second_;}
	std::vector<std::vector<cv::DMatch>> knn_matches() const {return knn_matches_;}
	void set_matches(std::vector<cv::DMatch> matches) {matches_=matches;}
	void set_matches(std::vector<cv::DMatch> matches, 
					std::vector<std::vector<cv::DMatch>> knnMatches) {matches_=matches; 
																	  knn_matches_= knnMatches;}
	std::vector<cv::DMatch> matches() const {return matches_;}

private:

	std::vector<cv::KeyPoint> keypoints_first_, keypoints_second_;
	std::vector<cv::DMatch> matches_;
	std::vector<std::vector<cv::DMatch>> knn_matches_;
};

struct FeatureInfo
{
	std::vector<cv::KeyPoint> keypoints_;
	cv::Mat descriptors_;
};

#endif // IMAGE_DATA_H_
