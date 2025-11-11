#ifndef PIPELINE_H_
#define PIPELINE_H_

#include <opencv2/opencv.hpp>

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

	clock_t previous_img_capture_time_ = 0;
	//Для siyi-a8 mini
	int camera_hfov_ = 81;

	float calculate_vertical_fov(float hfov_deg, int width, int height);
};

#endif // PIPELINE_H_
