#ifndef OPTICAL_FLOW_H_
#define OPTICAL_FLOW_H_

#include <opencv2/opencv.hpp>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sys/stat.h>

class OpticalFlowLkt
{
public:

	cv::Point2f get_optical_flow(const cv::Mat& curr_image, bool include_augmented_image = false,
                                 bool rev_flow = false);

private:
    
    cv::Mat prev_image_;
	double crop_factor_ = 0.6;

	// Параметры обнаружения углов Ши-Томаса
    int max_corners_ = 50;
    double quality_level_ = 0.2;
    double min_distance_ = 10;
    int block_size_ = 5;

	// Параметры для расчета оптического потока алгоритмом Лукаса-Канаде
    cv::Size win_size_ = cv::Size(15, 15);
    int max_level_ = 2;
    cv::TermCriteria criteria_ = cv::TermCriteria(cv::TermCriteria::EPS || cv::TermCriteria::COUNT, 20, 0.03);

    int iter_ = 0;
    std::string output_folder_;

    std::string create_output_folder();
    void vizualize_result(const cv::Mat& curr_image, std::vector<cv::Point2f> good_new,
                             std::vector<cv::Point2f> good_old);
};

#endif // OPTICAL_FLOW_H_