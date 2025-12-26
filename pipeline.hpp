#ifndef PIPELINE_H_
#define PIPELINE_H_

#include <opencv2/opencv.hpp>
#include <chrono>
#include <sys/stat.h>
#include <iomanip>
#include <fstream>
#include <unistd.h>
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

	int process_video(bool use_thermal_camera = false);

private:

	FrameProcessor frameProcessor_;
	MovesEstimator moves_estimator_;

	std::vector<ImageData> matched_data_;
	std::string outFile_;

	std::chrono::_V2::system_clock::time_point previous_img_capture_time_;
	//Для siyi-a8 mini 81
	//Для тепловизионной камеры ARKON 50
	//Для тепловизионной камеры ARKON 38.5526 при повороте на 90 градусов
	//Для веб-камеры A4Tech PK-333E 66
	float camera_hfov_ = 66;

	float tail_part_x_ = 0;
	float tail_part_y_ = 0;
	int percent_of_tail_for_use_ = 100;
	float tail_part_to_use_ = 100.0/percent_of_tail_for_use_;

	std::string output_folder_;
	int save_counter_ = 0;

	// тепловизионная камера ARKON
	std::string gstreamer_pipeline_thermal_ = "gst-launch-1.0 v4l2src device=/dev/video0 ! \
											 video/x-raw,format=YUY2 ! videoconvert ! appsink sync=false";
	// камера siyi-a8 mini
	// std::string gstreamer_pipeline_ = "gst-launch-1.0 rtspsrc location=\"rtsp://192.168.144.25:8554/main.264\"\
	// 								  latency=0 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false";
	// usb веб-камера A4Tech PK-333E 
	std::string gstreamer_pipeline_ = "gst-launch-1.0 v4l2src device=/dev/video2 ! video/x-raw,format=YUY2,framerate=30/1 ! videoconvert ! appsink sync=false";

	int fps = 30;
	int time_for_cap = 1000000/fps;

	float calculate_vertical_fov(float hfov_deg, int width, int height);
	void fix_thermal_camera_frame(cv::Mat& frame);
	std::string create_output_folder();
};

#endif // PIPELINE_H_
