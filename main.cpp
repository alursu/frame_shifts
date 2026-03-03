#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>

#include "pipeline.hpp"

int main(int argc, char* argv[])
{
	// Выводим только ошибки, остальные сообщения игнорируются
	cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_ERROR);

	// Стартовое пороговое значение для BRISK
	int threshold = 35;
	std::cout << "Threshold " << threshold << std::endl;

	// Создаем пайплайн
	Pipeline stitch (threshold);

	cv::Mat img1 = cv::imread("/home/teleskret/work/frame_shifts/build/arrows_2026-03-02_15-06-06/frame_000096.jpg");
	std::cout << img1.empty() << std::endl;
	// cv::imshow("img1", img1);
	// cv::waitKey(10000);
	cv::Mat img2 = cv::imread("/home/teleskret/work/frame_shifts/build/arrows_2026-03-02_15-06-06/frame_000130.jpg");
	// cv::imshow("img2", img2);
	// cv::waitKey(10000);
	std::cout << img2.empty() << std::endl;
    // Проверка размеров и типа
    // if (img1.size() != img2.size() || img1.type() != img2.type())
    //     return false;
    
    // Вычисляем норму L1 (сумма абсолютных разностей)
    // Если норма = 0 → изображения идентичны
    double ret = cv::norm(img1, img2, cv::NORM_L1);
	std::cout << ret << std::endl;

	// Запуск пайплайна
	stitch.process_video();
 	return 0;
}