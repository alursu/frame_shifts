#ifndef OPTICAL_FLOW_H_
#define OPTICAL_FLOW_H_

#include <opencv2/opencv.hpp>
#include <ctime>
#include <fstream>
#include <sys/stat.h>
#include <fstream>

class OpticalFlowLkt
{
public:

    OpticalFlowLkt();
    ~OpticalFlowLkt();
	cv::Point2f get_optical_flow(const cv::Mat& curr_image, bool rev_flow = false);

private:
    
    cv::Mat prev_image_;
	double crop_factor_ = 0.75;

	// Параметры обнаружения углов Ши-Томаса
    int max_corners_ = 50;
    double quality_level_ = 0.2;
    double min_distance_ = 10;
    int block_size_ = 5;

	// Параметры для расчета оптического потока алгоритмом Лукаса-Канаде
    cv::Size win_size_ = cv::Size(15, 15);
    int max_level_ = 2;
    cv::TermCriteria criteria_ = cv::TermCriteria(cv::TermCriteria::EPS || cv::TermCriteria::COUNT, 20, 0.03);

    // Параметры для прогнозирования смещений в моменты работы адаптивной коррекции
    int iter_frames_for_forecast_ = 0;
    bool calib_stopped = false;
    std::array<cv::Point2f, 25> forecast_displacements{};
    bool were_identic = false;

    // Параметры для сохранения изображений с результатами работы алгоритма
    int iter_ = 0;
    std::string output_folder_;

    // Параметры для фильтрации сопоставлений
    int max_count_of_comparisons_ = 20;
    int error_threshold_ = 30;

    std::string create_output_folder();
    void vizualize_result(const cv::Mat& curr_image, std::vector<cv::Point2f> good_new,
                             std::vector<cv::Point2f> good_old);
    void displacement_forecast(float flow_x, float flow_y);
    void save_image(const cv::Mat& img);
    cv::Point2f processing_calib_imgs (const cv::Mat &img);
};

#endif // OPTICAL_FLOW_H_