#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>

#include "pipeline.hpp"

int main(int argc, char* argv[])
{
	// Выводим только ошибки, остальные сообщения игнорируются
	cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_ERROR);

	// Имя файла для обработки (относительно директории с исполняемым файлом - frame_shifts.exe)
	std::string videoName = "test1.avi";
	
	// Имя файла для вывода информации о смещениях
	auto outName = "out.txt";
	std::cout << "Will write result in " << outName << std::endl;

	// Стартовое пороговое значение для BRISK
	int threshold = 35;
	std::cout << "Threshold " << threshold << std::endl;

	// Создаем пайплайн
	Pipeline stitch (threshold);

	// Запуск пайплайна
	stitch.ProcessVideo(videoName, outName);
 	return 0;
}