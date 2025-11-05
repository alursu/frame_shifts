#ifndef PIPELINE_H_
#define PIPELINE_H_

#include <opencv2/opencv.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <ctime>
#include <cmath>

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

	int ProcessVideo(std::string videoFileName, std::string outFileName);
protected:

	FrameProcessor m_frameProcessor;
	MovesEstimator m_estimator;
	// AutopilotInterface m_autopilot;
	// UartInterface *m_port;

	std::vector<ImageData> m_matchedData;
	std::string m_outFile;

	clock_t previous_img_capture_time = 0;
	//Для siyi-a8 mini
	int camera_hfov = 81;

	float CalculateVerticalFov(float hfov_deg, int width, int height);
};

void QuitHandler( int sig );

#endif
