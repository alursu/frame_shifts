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
	std::clog << "Threshold " << threshold << std::endl;

	// Создаем пайплайн
	Pipeline stitch (threshold);

	// Запуск пайплайна
	stitch.process_video(true);
 	return 0;
}