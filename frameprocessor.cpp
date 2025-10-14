#include "frameprocessor.hpp"

using namespace std;
using namespace cv;

FrameProcessor::FrameProcessor(std::string detectorName, int threshold, int planes)
	: m_threshold(threshold)
{
	// Cоздаем детектор, дескриптор и матчер. Threshold - пороговое значение
	// для определения точки как ключевой, planes - кол-во октав, т.е. кол-во
	// уровней в пирамиде - пирамида состоит из изображения и его копий
	// в масштабе 0.5 для каждого уровня (т.е. первый уровень - 0.5, второй - 0.25 и т.д.). 
	// Обеспечивает инвариантность алгоритма к масштабированию 
    m_pDetector = BRISK::create(threshold, planes);
    m_pDescriptor = BRISK::create(threshold, planes);

	// Т.к. BRISK бинарный (как и ORB), то используется NORM_HAMMING
	// SIFT и SURF используют NORM_L2
    m_pMatcher = BFMatcher::create(NORM_HAMMING, false);
}

FeatureInfo FrameProcessor::GetKeypointData(cv::Mat const &image)
{
	FeatureInfo frameInfo;

	// Определяем ключевые точки
	m_pDetector->detect(image, frameInfo.keypoints);

	// Вычисляем дескрипторы для особых точек
	m_pDescriptor->compute(image, frameInfo.keypoints, frameInfo.descriptors);
    return frameInfo;
}

ImageData FrameProcessor::MatchImages(cv::InputArray& im1, FeatureInfo & first, cv::InputArray& im2,
                                  FeatureInfo & second, double relPower)
{
	second = GetKeypointData(im2.getMat());
	ImageData ret = MatchDescriptors(first, second, relPower);

	// Пока кол-во отфильтрованных матчей не будет соответствовать 
	// указанному диапазону 90-360, меняем пороговое значение и 
	// повторно определяем ключевые точки и дескрипторы для нового значения
	while (!CheckThreshold(ret))
	{
		cout << "New threshold " << m_threshold << endl;
		first = GetKeypointData(im1.getMat());
		second = GetKeypointData(im2.getMat());
		ret = MatchDescriptors(first, second, relPower);
	}
	return ret;
}

int FrameProcessor::SetThreshold(int newThreshold)
{
	// Переопределяем пороговое значение
	cv::Ptr<cv::BRISK> ptr = m_pDetector.dynamicCast<cv::BRISK>();
	ptr->setThreshold(newThreshold);
	return ptr->getThreshold();
}

ImageData FrameProcessor::MatchDescriptors(FeatureInfo const &first, FeatureInfo const &second, double relPower)
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
	m_pMatcher->knnMatch(first.descriptors, second.descriptors, knnMatches,2);
	for (auto match : knnMatches)
	{
		if (match[0].distance < relPower * match[1].distance)
			goodMatches.push_back(match[0]);
	}
	ImageData retData;

	// Сохраняем отфильтрованные матчи
	retData.SetKeypoints(first.keypoints,second.keypoints);
	retData.SetMatches(goodMatches);
	return retData;
}

bool FrameProcessor::CheckThreshold(ImageData const & data)
{
	// Если матчей меньше 90 и пороговое значение больше 10, снижаем порог на величину, 
	// зависящую от степени отличия нынешнего кол-ва матчей и "центрального" значения из 
	// указанного диапазона матчей
	auto totalMatches = data.Matches().size();
	if (totalMatches < m_minimumFeaturesRequired && m_threshold>minimumThreshold)
	{
		m_threshold-= static_cast<int>(ceil((1-totalMatches/(2.5*m_minimumFeaturesRequired))*20));
		SetThreshold(m_threshold);
		return false;
	}

	// Аналогично, проверяем верхнуюю границу - 360. 
	if (totalMatches > 4 * m_minimumFeaturesRequired)
	{
		m_threshold+= static_cast<int>(ceil((1-(2.5*m_minimumFeaturesRequired)/totalMatches)*20));
		SetThreshold(m_threshold);
		return false;
	}
    return true;
}