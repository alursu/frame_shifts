#include "pipeline.hpp"

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
int Pipeline::process_video(bool use_thermal_camera)
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

	if (use_thermal_camera)
		cap->open(gstreamer_pipeline_thermal, cv::CAP_GSTREAMER);
	else 
		cap->open(gstreamer_pipeline, cv::CAP_GSTREAMER);

	// Если захват видео не удался - вывод сообщения и завершение программы
	if (!cap->isOpened()){ 
		std::clog << "Video source is not opened" << std::endl;
		cap.release();
		return -1;
	}

	// port->start();
	// autopilot->start();

	std::string output_folder = create_output_folder();
	int save_counter = 0;

	// Если захватили кадр - начинаем обработку
	if (cap->grab())
	{
		previous_img_capture_time_ = std::chrono::high_resolution_clock::now();
		// Загружаем изображение. Загружаем в second, чтобы далее сравнивать соседние кадры
		// Т.е. меняем second и first местами каждый раз, загружаем последующее изображение в 
		// second
		cap->retrieve(second);

		if (use_thermal_camera){
			fix_thermal_camera_frame(second);

			std::ostringstream oss;
    		oss << output_folder << "/frame_" << std::setfill('0') << std::setw(6) << save_counter++ << ".jpg";
    		cv::imwrite(oss.str(), second);
		}

		// Cоздаем шаблон, с разрешением на 10 пикселей меньше по высоте и ширине исходного
		cropRect = Rect(OFFSET_Y, OFFSET, second.cols-2*OFFSET_Y, second.rows-2*OFFSET);

		// Обрезаем исходное изображение по шаблону (по 5 пикселей с каждой стороны).
		// Т.к. наибольшие искажения наблюдаются в близи к краям изображения, то просто обрезаем их 
		second = Mat(second, cropRect);

		// Переводим в градацию серого
		cv::cvtColor(second,second,cv::COLOR_BGR2GRAY);

		// // Определяем ключевые точки изображения и соответствующие им дескрипторы
		// secondInfo = frameProcessor_.get_keypoint_data(second);

		shift = opticalflow.get_optical_flow(second);
	}

	int camera_vfov = calculate_vertical_fov(camera_hfov_, second.cols + 10, second.rows + 10);

	float pixels_per_radian_h = second.cols / (camera_hfov_*M_PI / 180);
    float pixels_per_radian_v = second.rows / (camera_vfov*M_PI / 180);

	// Пока можем захватывать кадры - обработка
	while (cap->grab())
	{
		auto frame_grabbed_time = std::chrono::high_resolution_clock::now();
		auto time_diff_btwn_capturing_imgs = std::chrono::duration_cast<std::chrono::microseconds>(frame_grabbed_time - previous_img_capture_time_);
		float diff_btwn_capturing_imgs_sec = time_diff_btwn_capturing_imgs.count()/1000000.0;

		first = second.clone();
		swap(firstInfo, secondInfo);
		cap->retrieve(second);

		// Если кадр оказался пустым, пропускаем итерацию
		if (second.rows == 0 || second.cols == 0){
			continue;
		}

		if (use_thermal_camera){
			fix_thermal_camera_frame(second);

			std::ostringstream oss;
    		oss << output_folder << "/frame_" << std::setfill('0') << std::setw(6) << save_counter++ << ".jpg";
    		cv::imwrite(oss.str(), second);
		}

		second = Mat(second, cropRect);
		cv::cvtColor(second,second,cv::COLOR_BGR2GRAY);

		// // Сравниваем соседние кадры
		// auto result = frameProcessor_.match_images(first, firstInfo, second, secondInfo);

		// // Получаем на основе сравнения матрицу афинных преобразований
		// cv::Mat move = moves_estimator_.estimate_movements(result);

		shift = opticalflow.get_optical_flow(second);

		float flow_rate_x = shift.x / (pixels_per_radian_h * (diff_btwn_capturing_imgs_sec));
		float flow_rate_y = shift.y / (pixels_per_radian_v * (diff_btwn_capturing_imgs_sec));

		tail_part_x+= (shift.x - (int)shift.x);
		tail_part_y+= (shift.y - (int)shift.y);
		if (abs(tail_part_x) > tail_part_to_use){
			shift.x+=(tail_part_x > 0) ? 1 : -1;
			tail_part_x+=(tail_part_x > 0) ? -tail_part_to_use : tail_part_to_use;
		}
		if(abs(tail_part_y) > tail_part_to_use){
			shift.y+=(tail_part_y > 0) ? 1 : -1;
			tail_part_y+=(tail_part_y > 0) ? -tail_part_to_use : tail_part_to_use;
		}

		autopilot->write_optical_flow(shift.x, shift.y, flow_rate_x, flow_rate_y);
		std::clog << "x shifts: " << shift.x << "  " << "y shifts: " << shift.y << std::endl;

		previous_img_capture_time_ = frame_grabbed_time;
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
        float aspect_ratio = (float)height / width;

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

void Pipeline::fix_thermal_camera_frame(cv::Mat& frame) {

	if (frame.empty() || frame.cols <= 1) 
		return;

	//Меняем местами правую и левую половины кадра
	int half = frame.cols / 2;
	cv::Mat left = frame(cv::Rect(0, 0, half, frame.rows));
	cv::Mat right = frame(cv::Rect(half, 0, half, frame.rows));

	frame.release();
	cv::hconcat(right, left, frame); // Меняем местами

	int half_height = frame.rows / 2;

	// Оставляем нижнюю половину (основную часть)
	cv::Rect main_rect(0, half_height+8, frame.cols, half_height-8);
	frame = frame(main_rect);

	return;
}

std::string Pipeline::create_output_folder() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&time);
    
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    std::string folder_name = oss.str();
    
    mkdir(folder_name.c_str(), 0777);
    return folder_name;
}