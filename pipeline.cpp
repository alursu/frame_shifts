#include "pipeline.hpp"
#include <chrono>

#define OFFSET 5
#define OFFSET_Y 5

using namespace std;
using namespace cv;

// Глобальные переменные и вспомогательная функция для обработки прерывания
std::shared_ptr<AutopilotInterface> autopilot_interface_quit_;
std::shared_ptr<UartInterface> port_quit_;
void quit_handler( int sig );


// Cоздаем обработчик m_frameProcessor для детекции, сравнения особых точек
Pipeline::Pipeline(int threshold, int octaves)
	: frameProcessor_("BRISK", threshold, octaves)
{

}

// Основной пайплайн
int Pipeline::process_video()
{
	Mat first, second;

	// FeatureInfo - структура для хранения ключевых точек и их дескрипторов 
	FeatureInfo firstInfo, secondInfo;

	cv::Rect cropRect;
	cv::Point2f shift;
	OpticalFlowLkt opticalflow;

	std::shared_ptr<UartInterface> port = std::make_shared<UartInterface>("/dev/ttyACM0", 115200);
	std::shared_ptr<AutopilotInterface> autopilot = std::make_shared<AutopilotInterface>(port);

	port_quit_ = port;
	autopilot_interface_quit_ = autopilot;
	signal(SIGINT,quit_handler);

	// Захват видео
	Ptr<VideoCapture> cap = Ptr<VideoCapture>(new VideoCapture());

	// std::string gstreamer_pipeline_teplak = "gst-launch-1.0 v4l2src device=/dev/video2 ! video/x-raw,width=640,height=1032,format=YUY2 ! videoconvert ! appsink sync=false";
	// cap->open(gstreamer_pipeline_teplak, cv::CAP_GSTREAMER);

	std::string gstreamer_pipeline = "gst-launch-1.0 rtspsrc location=\"rtsp://192.168.144.25:8554/main.264\" latency=0 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false";
	cap->open(gstreamer_pipeline, cv::CAP_GSTREAMER);
	// cap->open("../test1.avi");

	// Если захват видео не удался - вывод сообщения и завершение программы
	if (!cap->isOpened()){ 
		std::clog << "Video source is not opened" << std::endl;
		cap.release();
		return -1;
	}

	port->start();
	autopilot->start();

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

		// // Определяем ключевые точки изображения и соответствующие им дескрипторы
		// secondInfo = frameProcessor_.GetKeypointData(second);

		shift = opticalflow.get_optical_flow(second);
	}

	int camera_vfov = calculate_vertical_fov(camera_hfov_, second.cols, second.rows);

	float pixels_per_radian_h = second.cols / (camera_hfov_*M_PI / 180);
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

		second = Mat(second, cropRect);
		cv::cvtColor(second,second,cv::COLOR_BGR2GRAY);

		// // Сравниваем соседние кадры
		// auto result = frameProcessor_.MatchImages(first, firstInfo, second, secondInfo);

		// // Получаем на основе сравнения матрицу афинных преобразований
		// cv::Mat move = moves_estimator_.EstimateMovements(result);

		shift = opticalflow.get_optical_flow(second);

		float flow_rate_x = shift.x / (pixels_per_radian_h * (start - previous_img_capture_time_));
		float flow_rate_y = shift.y / (pixels_per_radian_v * (start - previous_img_capture_time_));

		autopilot->write_optical_flow(shift.x, shift.y, flow_rate_x, flow_rate_y);

		std::clog << "x shifts: " << shift.x << "  " << "y shifts: " << shift.y << std::endl;
	}

	// Закрываем файлы и источник видео
	port->stop();
	autopilot->stop();
	cap->release();

	return 0;
}


float Pipeline::calculate_vertical_fov(float hfov_deg, int width, int height) {
	/**
	* Рассчитывает вертикальный FOV на основе горизонтального FOV и разрешения изображения
	* tan(vfov/2) = tan(hfov/2) * (height/weight)
	*/

	// Проверка входных данных для предотвращения математических ошибок
    if (width <= 0) 
        return 0;
    if (height <= 0)
        return 0;
    if (hfov_deg <= 0 || hfov_deg >= 180)
        return 0;

    try {
        float hfov_rad = hfov_deg * M_PI / 180;
        float aspect_ratio = height / width;

        float vfov_rad = 2 * std::atan(std::tan(hfov_rad / 2) * aspect_ratio);
        float vfov_deg = vfov_rad * 180 / M_PI;

        if (vfov_deg <= 0 || vfov_deg >= 180) {
            return 0;
        }

        return vfov_deg;
    } 
	catch (const std::exception& e) {
		std::cerr << "Mathematical error calculating VFOV" << std::endl;
        return 0;
    }
}


void quit_handler(int sig)
{
	std::clog << std::endl << "TERMINATING AT USER REQUEST" << std::endl;

	try {
		autopilot_interface_quit_->handle_quit(sig);
	}
	catch (int error){}

	try {
		port_quit_->stop();
	}
	catch (int error){}

	// Завершение программы
	exit(0);
}