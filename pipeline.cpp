#include "pipeline.hpp"
#include <ctime>

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
	VideoCapture cap(videoFileName);

	// Если захват видео не удался - вывод сообщения и завершение программы
	if (!cap.isOpened()){ 
		std::cout << "video is not opened" << std::endl;
		return -1;
	}
	cv::Rect cropRect;
	cv::Point2f out;
	Opticalflow opticalflow;

	// Если захватили кадр - начинаем обработку
	if (cap.grab())
	{
		// Загружаем изображение. Загружаем в second, чтобы далее сравнивать соседние кадры
		// Т.е. меняем second и first местами каждый раз, загружаем последующее изображение в 
		// second
		cap >> second;

		// Cоздаем шаблон, с разрешением на 10 пикселей меньше по высоте и ширине исходного
		cropRect = Rect(OFFSET_Y, OFFSET, second.cols-2*OFFSET_Y, second.rows-2*OFFSET);

		// Обрезаем исходное изображение по шаблону (по 5 пикселей с каждой стороны).
		// Т.к. наибольшие искажения наблюдаются в близи к краям изображения, то просто обрезаем их 
		second = Mat(second, cropRect);

		// Переводим в градацию серого
		cv::cvtColor(second,second,cv::COLOR_BGR2GRAY);

		// Определяем ключевые точки изображения и соответствующие им дескрипторы
		secondInfo = m_frameProcessor.GetKeypointData(second);

		clock_t start = clock();
		out = opticalflow.GetOpticalFlow(second);
		clock_t end = clock();
		time_lk << (end-start) << std::endl;
		shifts_lk << "x shifts: " << out.x << "  " << "y shifts: " << out.y << std::endl;
	}

	// Инициализируем потоки вывода для смещений и информации о времени работы программы
	std::ofstream time;
	time.open("time_test1_90_4_threshold_optimize_middle(20).txt");
	time << "Time before start matching: " << clock() << std::endl << "Start matching:" << std::endl;

	// Переменные для вычисления кол-ва кадров и суммарного времени обработки кадров (начиная со 2-го)
	int sum = 0;
	int frames = 0;

	int sum_lk = 0;

	// Пока можем захватывать кадры - обработка
	while (cap.grab())
	{
		clock_t start = clock();
		first = second.clone();
		swap(firstInfo, secondInfo);
		cap >> second;
		// Если кадр оказался пустым, пропускаем итерацию
		if (second.rows == 0 || second.cols == 0){
			continue;
		}

		second = Mat(second, cropRect);
		cv::cvtColor(second,second,cv::COLOR_BGR2GRAY);

		// Сравниваем соседние кадры
		auto result = m_frameProcessor.MatchImages(first, firstInfo, second, secondInfo);

		// Получаем на основе сравнения матрицу афинных преобразований
		cv::Mat move = m_estimator.EstimateMovements(result);

		// Записываем время обработки и смещения
		clock_t end = clock();
		time << (end-start) << std::endl;
		sum+=(end-start);
		frames++;
		shifts << "x shift: " << move.at<double>(0,2) << ", y shift: " << move.at<double>(1,2) <<std::endl;

		clock_t start_lk = clock();
		out = opticalflow.GetOpticalFlow(second);
		clock_t end_lk = clock();
		time_lk << (end_lk - start_lk) << std::endl;
		sum_lk+=(end_lk - start_lk);

		shifts_lk << "x shifts: " << out.x << "  " << "y shifts: " << out.y << std::endl;
	}

	time << "Общее время обработки: " << sum << ", Среднее время обработки: " << (float)(sum/frames) << ", Кол-во кадров: " << frames << std::endl;

	time_lk <<"Общее время обработки: " << sum_lk << ", Среднее время обработки: " << (float)(sum_lk/frames) << ", Кол-во кадров: " << frames << std::endl;
	// Закрываем файлы и источник видео
	cap.release();
	shifts.close();
	shifts_lk.close();
	time.close();
	time_lk.close();

	return 0;
}