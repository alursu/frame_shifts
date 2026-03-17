#ifndef PIPELINE_H_
#define PIPELINE_H_

#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <chrono>

// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui.hpp>
// #include <ctime>
// #include <cmath>

#include "image_data.hpp"
#include "frame_processor.hpp"
#include "moves_estimator.hpp"
#include "optical_flow_lkt.hpp"
#include "autopilot_interface.hpp"
#include "uart_interface.hpp"
#include "camera_interface.hpp"

class Pipeline
{
public:

	explicit Pipeline(int threshold = 30, int octaves = 3);

	int process_video();

protected:

	FrameProcessor frameProcessor_;
	MovesEstimator moves_estimator_;

	std::vector<ImageData> matched_data_;
	std::string outFile_;

	std::string output_folder_;
	int save_counter_ = 0;

	// clock_t previous_img_capture_time_ = 0;
	std::chrono::_V2::system_clock::time_point previous_img_capture_time_;
	
	//Для siyi-a8 mini
	int camera_hfov_ = 48.3;

	float tail_part_x_ = 0;
	float tail_part_y_ = 0;
	int percent_of_tail_for_use_ = 50;
	float tail_part_to_use_ = 100.0/percent_of_tail_for_use_;

	float calculate_vertical_fov(float hfov_deg, int width, int height);
	std::string create_output_folder();
};

#endif // PIPELINE_H_
