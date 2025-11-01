#include "pipeline.hpp"
#include <ctime>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/time.h>

#define OFFSET 5
#define OFFSET_Y 5

using namespace std;
using namespace cv;

#define M_PI 3.1415926535897932384626433832795

// Cоздаем обработчик m_frameProcessor для детекции, сравнения особых точек
Pipeline::Pipeline(int threshold, int octaves)
	: m_frameProcessor("BRISK", threshold, octaves)
{
}

// Основной пайплайн
int Pipeline::ProcessVideo(std::string videoFileName, std::string outFileName)
{
	Mat first, second;

	std::ofstream shifts;
	shifts.open(outFileName);

	std::ofstream shifts_lk;
	shifts_lk.open("shifts_lk.txt");

	std::ofstream time_lk;
	time_lk.open("time_lk.txt");

	// FeatureInfo - структура для хранения ключевых точек и их дескрипторов 
	FeatureInfo firstInfo, secondInfo;

	// Захват видео
	// VideoCapture cap(videoFileName);
	Ptr<VideoCapture> cap = Ptr<VideoCapture>(new VideoCapture());
	std::string gstreamer_pipeline = "gst-launch-1.0 rtspsrc location=\"rtsp://192.168.144.25:8554/main.264\" latency=0 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false";
	cap->open(gstreamer_pipeline, cv::CAP_GSTREAMER);
	

	// Если захват видео не удался - вывод сообщения и завершение программы
	if (!cap->isOpened()){ 
		std::cout << "video is not opened" << std::endl;
		cap->release();
		return -1;
	}
	cv::Rect cropRect;
	cv::Point2f out;
	OpticalFlowLkt opticalflow;

	m_port = new UartInterface();

	// Если захватили кадр - начинаем обработку
	if (cap->grab())
	{
		// Загружаем изображение. Загружаем в second, чтобы далее сравнивать соседние кадры
		// Т.е. меняем second и first местами каждый раз, загружаем последующее изображение в 
		// second
		*cap >> second;

		// Cоздаем шаблон, с разрешением на 10 пикселей меньше по высоте и ширине исходного
		cropRect = Rect(OFFSET_Y, OFFSET, second.cols-2*OFFSET_Y, second.rows-2*OFFSET);

		// Обрезаем исходное изображение по шаблону (по 5 пикселей с каждой стороны).
		// Т.к. наибольшие искажения наблюдаются в близи к краям изображения, то просто обрезаем их 
		second = Mat(second, cropRect);

		// Переводим в градацию серого
		cv::cvtColor(second,second,cv::COLOR_BGR2GRAY);

		// Определяем ключевые точки изображения и соответствующие им дескрипторы
		secondInfo = m_frameProcessor.GetKeypointData(second);

		clock_t start = 1000*clock()/CLOCKS_PER_SEC;
		out = opticalflow.GetOpticalFlow(second);
		clock_t end = 1000*clock()/CLOCKS_PER_SEC;
		time_lk << (end-start) << std::endl;
	}

	// Инициализируем потоки вывода для смещений и информации о времени работы программы
	std::ofstream time;
	time.open("time_test1_90_4_threshold_optimize_middle(20).txt");
	time << "Time before start matching: " << clock() << std::endl << "Start matching:" << std::endl;

	// Переменные для вычисления кол-ва кадров и суммарного времени обработки кадров (начиная со 2-го)
	int sum = 0;
	int frames = 0;

	int sum_lk = 0;

	int camera_vfov = CalculateVerticalFov(camera_hfov, second.cols, second.rows);

	float pixels_per_radian_h = second.cols / (camera_hfov*M_PI / 180);
    float pixels_per_radian_v = second.rows / (camera_vfov*M_PI / 180);

	// Пока можем захватывать кадры - обработка
	while (cap->grab())
	{
		clock_t start = 1000*clock()/CLOCKS_PER_SEC;
		first = second.clone();
		swap(firstInfo, secondInfo);
		*cap >> second;
		// Если кадр оказался пустым, пропускаем итерацию
		if (second.rows == 0 || second.cols == 0){
			continue;
		}
		
		cv::imshow("stream", second);
		cv::waitKey(30);	

		second = Mat(second, cropRect);
		cv::cvtColor(second,second,cv::COLOR_BGR2GRAY);

		// Сравниваем соседние кадры
		// auto result = m_frameProcessor.MatchImages(first, firstInfo, second, secondInfo);

		// Получаем на основе сравнения матрицу афинных преобразований
		// cv::Mat move = m_estimator.EstimateMovements(result);

		// Записываем время обработки и смещения
		clock_t end = 1000*clock()/CLOCKS_PER_SEC;
		time << (end-start) << std::endl;
		sum+=(end-start);
		frames++;
		// shifts << "x shift: " << move.at<double>(0,2) << ", y shift: " << move.at<double>(1,2) <<std::endl;

		clock_t start_lk = 1000*clock()/CLOCKS_PER_SEC;
		out = opticalflow.GetOpticalFlow(second);
		clock_t end_lk = 1000*clock()/CLOCKS_PER_SEC;
		time_lk << (end_lk - start_lk) << std::endl;
		sum_lk+=(end_lk - start_lk);

		float flow_rate_x = out.x / (pixels_per_radian_h * (start - previous_img_capture_time));
		float flow_rate_y = out.y / (pixels_per_radian_v * (start - previous_img_capture_time));

		// m_uartMAVlink.SendOpticalFlow(out.x, out.y, flow_rate_x, flow_rate_y);

		std::cout << "x shifts: " << out.x << "  " << "y shifts: " << out.y << std::endl;
		shifts_lk << "x shifts: " << out.x << "  " << "y shifts: " << out.y << std::endl;

		// cv::imshow("stream", second);
		// cv::waitKey(30);	
	}

	time << "Общее время обработки: " << sum << ", Среднее время обработки: " << (float)(sum/frames) << ", Кол-во кадров: " << frames << std::endl;

	time_lk <<"Общее время обработки: " << sum_lk << ", Среднее время обработки: " << (float)(sum_lk/frames) << ", Кол-во кадров: " << frames << std::endl;
	// Закрываем файлы и источник видео
	cap->release();
	shifts.close();
	shifts_lk.close();
	time.close();
	time_lk.close();

	return 0;
}

float Pipeline::CalculateVerticalFov(float hfov_deg, int width, int height) {
    /**
     * Calculate vertical FOV based on horizontal FOV, image width, and height
     * tan(vfov/2) = tan(hfov/2) * (height/width)
     */

    // logging prefix for all messages from this function
    std::string logging_prefix_str = "calculate_vertical_fov:";

    // Validate inputs to prevent mathematical errors
    if (width <= 0) {
        // logger.error(logging_prefix_str + " invalid image width: " + std::to_string(width));
        return 0;
    }

    if (height <= 0) {
        // logger.error(logging_prefix_str + " invalid image height: " + std::to_string(height));
        return 0;
    }

    if (hfov_deg <= 0 || hfov_deg >= 180) {
        // logger.error(logging_prefix_str + " invalid horizontal FOV: " + std::to_string(hfov_deg) + " (must be between 0 and 180)");
        return 0;
    }

    try {
        // Convert horizontal FOV from degrees to radians
        float hfov_rad = hfov_deg * M_PI / 180;

        // Calculate aspect ratio
        float aspect_ratio = static_cast<float>(height) / static_cast<float>(width);

        // Use trigonometric relationship to calculate vertical FOV
        float vfov_rad = 2 * std::atan(std::tan(hfov_rad / 2) * aspect_ratio);

        // Convert back to degrees
        float vfov_deg = vfov_rad * 180 / M_PI;

        // Sanity check result
        if (vfov_deg <= 0 || vfov_deg >= 180) {
            // logger.error(logging_prefix_str + " calculated invalid VFOV: " + std::to_string(vfov_deg));
            return 0;
        }

        return vfov_deg;
    } catch (const std::exception& e) {
        // logger.error(logging_prefix_str + " mathematical error calculating VFOV: " + std::string(e.what()));
        return 0;
    }
}

void Pipeline::QuitHandler(int sig)
{

	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// port
	try {
		port_quit->Stop();
	}
	catch (int error){}

	// end program here
	exit(0);

}
