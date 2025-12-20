#ifndef FRAME_PROCESSOR_H_
#define FRAME_PROCESSOR_H_

#include <opencv2/features2d.hpp>
#include "image_data.hpp"
#include <cmath>

class FrameProcessor
{
public:

    FrameProcessor(std::string detectorName, int threshold, int planes);

    FeatureInfo get_keypoint_data(cv::Mat const & image);
    ImageData  match_images(cv::InputArray& im1, FeatureInfo & first, cv::InputArray& im2,
                            FeatureInfo & second, double relPower = 0.7);

    int set_threshold(int newThreshold);

private:

    ImageData  match_descriptors(FeatureInfo const & first, 
                                 FeatureInfo const & second, double relPower = 0.7);
    bool check_threshold(ImageData const & data);

    void draw_keypoints_and_lines (cv::Mat &first_img, cv::Mat &second_img, 
                                   FeatureInfo const &first, FeatureInfo const &second, 
                                   std::vector<cv::DMatch> &goodMatches, std::string window_name);
    void draw_arrow_lines (cv::Mat& curr_image, FeatureInfo const &first,
                           FeatureInfo const &second, std::vector<cv::DMatch> matches);

	cv::Ptr<cv::FeatureDetector> detector_;
	cv::Ptr<cv::DescriptorExtractor> descriptor_;
	cv::Ptr<cv::DescriptorMatcher> matcher_;

    double minimum_features_required_ = 90;
    float constexpr static minimum_threshold_ = 5.0;
    int threshold_;
    int gain_for_step_;
    int previous_matches_size_;
    int iter_ = 0;
};

#endif // FRAME_PROCESSOR_H_