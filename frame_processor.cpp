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

	previous_matches_size_ = ret.matches().size();
	gain_for_step_ = 20;

	// Пока кол-во отфильтрованных матчей не будет соответствовать 
	// указанному диапазону 90-360, меняем пороговое значение и 
	// повторно определяем ключевые точки и дескрипторы для нового значения
	while (!check_threshold(ret))
	{
		clog << "New threshold " << threshold_ << endl;
		first = get_keypoint_data(im1.getMat());
		second = get_keypoint_data(im2.getMat());
		ret = match_descriptors(first, second, relPower);
		if (threshold_ == minimum_threshold_)
			break;
	} 
	// Раскомментировать для отображения результатов работы алгоритма
	// cv::Mat img = im2.getMat();
	// draw_arrow_lines(img, first, second, ret.matches());
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
	ImageData retData;

	matcher_->knnMatch(first.descriptors_, second.descriptors_, knnMatches,2);

	// Если количество матчей 0 или 1, возвращаем данные без фильтрации  
	if (knnMatches.size() < 2){
		retData.set_keypoints(first.keypoints_,second.keypoints_);
		retData.set_matches(goodMatches);
		return retData;
	}

	for (auto match : knnMatches)
	{
		//Если для дескриптора особой точки одного изображения не было найдено ни
		// одного сопоставления, пропускаем итерацию и переходим к следующей точке.
		if (match.empty())
			continue;

		if (match[0].distance < relPower * match[1].distance)
			goodMatches.push_back(match[0]);
	}

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
	// Также для избежания "хождения по кругу" из одних и тех же значений, не удовлетворяющих
	// условиям, редактируется коэффициент gain_for_step для порогового значения
	auto totalMatches = data.matches().size();

	// Если нет кол-ва матчей, удовлятворяющих установленному диапазону,
	// выбираем пороговое значение, при котором кол-во матчей превышает диапазон
	if (gain_for_step_ == 1 && totalMatches > 4 * minimum_features_required_)
		return true;

	float percent_diff_to_middle = totalMatches/(2.5*minimum_features_required_);
	if (percent_diff_to_middle > 1)
		percent_diff_to_middle = pow(percent_diff_to_middle, -1); 

	if (totalMatches < minimum_features_required_)
	{
		if (previous_matches_size_ > 4 * minimum_features_required_)
			gain_for_step_ = ceil(gain_for_step_/2.0);

		threshold_-= static_cast<int>(ceil((1-percent_diff_to_middle)*gain_for_step_));

		if (threshold_ < minimum_threshold_)
			threshold_ = minimum_threshold_;

		set_threshold(threshold_);
		previous_matches_size_ = totalMatches;
		return false;
	}

	// Аналогично, проверяем верхнуюю границу - 360. 
	if (totalMatches > 4 * minimum_features_required_)
	{
		if (previous_matches_size_ < minimum_features_required_)
			gain_for_step_ = ceil(gain_for_step_/2.0);

		threshold_+= static_cast<int>(ceil((1-percent_diff_to_middle)*gain_for_step_));
		set_threshold(threshold_);
		previous_matches_size_ = totalMatches;
		return false;
	}
    return true;
}

void FrameProcessor::draw_keypoints_and_lines(cv::Mat &first_img, cv::Mat &second_img, 
											  FeatureInfo const &first, FeatureInfo const &second, 
											  std::vector<cv::DMatch> &goodMatches, std::string window_name)
{
    cv::Mat img;
    cv::Mat img_2;
    cv::Mat result;

    cv::drawKeypoints(first_img, first.keypoints_, img);
    cv::drawKeypoints(second_img, second.keypoints_, img_2);
    cv::drawMatches(img, first.keypoints_, img_2, second.keypoints_, goodMatches, result);

    cv::namedWindow(window_name, cv::WINDOW_KEEPRATIO);
    cv::imshow(window_name, result);
    cv::resizeWindow(window_name, 600, 600);
    cv::waitKey();
}

void FrameProcessor::draw_arrow_lines(cv::Mat &curr_image, FeatureInfo const &first, 
									  FeatureInfo const &second, std::vector<cv::DMatch> matches)
{
	cv::Mat img = curr_image.clone();
    for (auto const & match : matches)
    {
		Point2f end_point = Point2f(first.keypoints_[match.queryIdx].pt);
		Point2f start_point = Point2f(second.keypoints_[match.trainIdx].pt);
		cv::arrowedLine(img, start_point, end_point, cv::Scalar(0), 1, 8, 0, 1);
    }

	// Раскомментировать для демонстрации работы во время работы программы
    // P.S.: Всплывающие окна закрывать нажатием любой клавиши
    // cv::imshow("BRISK result", img);
    // cv::waitKey(0);

    // Раскомментировать для сохранения результатов в папку result
    std::stringstream result_image_name;
    result_image_name << "./result_brisk_siyi100/" << "/frame_" << std::setfill('0') << std::setw(6) << iter_++ << ".jpg";
    cv::imwrite(result_image_name.str(), img);

    return;
}
