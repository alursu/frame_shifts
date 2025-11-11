#include "frame_processor.hpp"

using namespace std;
using namespace cv;

FrameProcessor::FrameProcessor(std::string detectorName, int threshold, int planes)
	: threshold_(threshold)
{
	// Cоздаем детектор, дескриптор и матчер. Threshold - пороговое значение
	// для определения точки как ключевой, planes - кол-во октав, т.е. кол-во
	// уровней в пирамиде - пирамида состоит из изображения и его копий
	// в масштабе 0.5 для каждого уровня (т.е. первый уровень - 0.5, второй - 0.25 и т.д.). 
	// Обеспечивает инвариантность алгоритма к масштабированию 
    detector_ = BRISK::create(threshold, planes);
    descriptor_ = BRISK::create(threshold, planes);

	// Т.к. BRISK бинарный (как и ORB), то используется NORM_HAMMING
	// SIFT и SURF используют NORM_L2
    matcher_ = BFMatcher::create(NORM_HAMMING, false);
}

FeatureInfo FrameProcessor::get_keypoint_data(cv::Mat const &image)
{
	FeatureInfo frameInfo;

	// Определяем ключевые точки
	detector_->detect(image, frameInfo.keypoints_);

	// Вычисляем дескрипторы для особых точек
	descriptor_->compute(image, frameInfo.keypoints_, frameInfo.descriptors_);
    return frameInfo;
}

ImageData FrameProcessor::match_images(cv::InputArray& im1, FeatureInfo & first, cv::InputArray& im2,
                                  FeatureInfo & second, double relPower)
{
	second = get_keypoint_data(im2.getMat());
	ImageData ret = match_descriptors(first, second, relPower);

	// Пока кол-во отфильтрованных матчей не будет соответствовать 
	// указанному диапазону 90-360, меняем пороговое значение и 
	// повторно определяем ключевые точки и дескрипторы для нового значения
	while (!check_threshold(ret))
	{
		cout << "New threshold " << threshold << endl;
		first = get_keypoint_data(im1.getMat());
		second = get_keypoint_data(im2.getMat());
		ret = match_descriptors(first, second, relPower);
	}
	return ret;
}

int FrameProcessor::set_threshold(int new_threshold)
{
	// Переопределяем пороговое значение
	cv::Ptr<cv::BRISK> ptr = detector_.dynamicCast<cv::BRISK>();
	ptr->setThreshold(new_threshold);
	return ptr->getThreshold();
}

ImageData FrameProcessor::match_descriptors(FeatureInfo const &first, FeatureInfo const &second, double relPower)
{ 
	// определение матчей - сопоставление особых точек по их дескрипторам
	// + фильтрация c помощью теста Лоу:
	// Тест Лоу отсеивает ложные совпадения путем сравнения расстояния до
	// лучшего матча (лучшая точка соответствия на втором изображении) d1 и второго
	// (следующего в списке) лучшего матча d2. Если d1 намного меньше d2
	// (d1/d2 <0.7…0.8 - определяется relPower), то лучшая точка считается уникальной, 
	// а матч записывается в goodMatches, иначе сопоставление расценивается как случайное совпадение.
	vector<cv::DMatch> goodMatches;
	std::vector<std::vector<cv::DMatch>> knnMatches;
	matcher_->knnMatch(first.descriptors_, second.descriptors_, knnMatches,2);
	for (auto match : knnMatches)
	{
		if (match[0].distance < relPower * match[1].distance)
			goodMatches.push_back(match[0]);
	}
	ImageData retData;

	// Сохраняем отфильтрованные матчи
	retData.set_keypoints(first.keypoints_,second.keypoints_);
	retData.set_matches(goodMatches);
	return retData;
}

bool FrameProcessor::check_threshold(ImageData const & data)
{
	// Если матчей меньше 90 и пороговое значение больше 10, снижаем порог на величину, 
	// зависящую от степени отличия нынешнего кол-ва матчей и "центрального" значения из 
	// указанного диапазона матчей
	auto totalMatches = data.matches().size();
	if (totalMatches < minimum_features_required_ && threshold_>minimum_threshold_)
	{
		threshold_-= static_cast<int>(ceil((1-totalMatches/(2.5*minimum_features_required_))*20));
		if (threshold_ < minimum_threshold_)
			threshold_ = minimum_threshold_;
		set_threshold(threshold_);
		return false;
	}

	// Аналогично, проверяем верхнуюю границу - 360. 
	if (totalMatches > 4 * minimum_features_required_)
	{
		threshold_+= static_cast<int>(ceil((1-(2.5*minimum_features_required_)/totalMatches)*20));
		set_threshold(threshold_);
		return false;
	}
    return true;
}