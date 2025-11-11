#include "optical_flow_lkt.hpp"

cv::Point2f OpticalFlowLkt::get_optical_flow(const cv::Mat& curr_image, bool include_augmented_image, 
                                             bool rev_flow) {

    if (curr_image.empty()) {
        return cv::Point2f(0,0);
         //что-то нужно с этим сделать, как-то обработать
    }   

    cv::Mat curr_image_grey = curr_image.clone();

    // Оптимизация - используем только центральные 60% изображения для обработки
    // Мб использовать и для BRISK? Но с меньшим процентом. Мб переписать без этой кучи переменных
    int h = curr_image_grey.rows;
    int w = curr_image_grey.cols;
    int crop_h = static_cast<int>(h * crop_factor_);
    int crop_w = static_cast<int>(w * crop_factor_);
    int start_y = (h - crop_h) / 2;
    int start_x = (w - crop_w) / 2;
    cv::Rect roi(start_x, start_y, crop_w, crop_h);
    curr_image_grey = curr_image_grey(roi);

    // Оптимизация со снижением разрешения в 2 раза? Реально ли так отработает?
    cv::resize(curr_image_grey, curr_image_grey, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);

    if (prev_image_.empty()) {
        prev_image_ = curr_image_grey.clone();
        return cv::Point2f(0,0);
        // надо обработать
    }

    std::vector<cv::Point2f> corners0;

    cv::goodFeaturesToTrack(prev_image_, corners0, max_corners_, quality_level_, 
                            min_distance_, cv::Mat(), block_size_);

    if (corners0.empty()) {
        prev_image_ = curr_image_grey.clone();
        return cv::Point2f(0,0);
        // надо обработать
    }

    std::vector<cv::Point2f> corners1;
    std::vector<uchar> st;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK(prev_image_, curr_image_grey, corners0, corners1, st, err,
                             win_size_, max_level_, criteria_);

    int good_count = 0;
    for (auto s : st) good_count += s;

    if (good_count < 5) {
        cv::Size retryWinSize(12, 12);
        int retryMaxLevel = 1;
        cv::calcOpticalFlowPyrLK(prev_image_, curr_image_grey, corners0, corners1, st, 
                                 err, retryWinSize, retryMaxLevel);
    }

    if (rev_flow) {
        std::vector<cv::Point2f> rev_corners1;
        std::vector<uchar> stRev;
        std::vector<float> errRev;
        cv::calcOpticalFlowPyrLK(curr_image_grey, prev_image_, corners1, rev_corners1, 
                                 stRev, errRev, cv::Size(12, 12), 1, criteria_);

        for (size_t i = 0; i < corners0.size(); ++i) {
            if (i >= rev_corners1.size() || i >= stRev.size()) break;
            double dist = cv::norm(corners0[i] - rev_corners1[i]);
            st[i] = st[i] & stRev[i] & (dist <= 0.5);
        }
    }

    prev_image_ = curr_image_grey.clone();

    std::vector<cv::Point2f> good_new;
    std::vector<cv::Point2f> good_old;
    std::vector<float> good_errors;
    for (size_t i = 0; i < st.size(); ++i) {
        if (st[i]) {
            good_new.push_back(corners1[i]);
            good_old.push_back(corners0[i]);
            good_errors.push_back(err[i]);
        }
    }

    if (good_new.empty()) {
        return cv::Point2f(0,0);
        // надо обработать
    }

    std::vector<cv::Point2f> flow_vectors;
    for (size_t i = 0; i < good_new.size(); ++i) {
        flow_vectors.push_back(good_new[i] - good_old[i]);
    }

    // Рассчет веса = 1.0 / (good_errors + 1e-6)
    std::vector<double> weights;
    weights.reserve(good_errors.size());
    for (int i = 0; i < good_errors.size(); ++i) {
        double val = good_errors[i];
        weights.push_back(1.0 / (val + 1e-6));
    }

    // Расчет взвешенных средних flow_x и flow_y
    float flow_x = 0.0;
    float flow_y = 0.0;
    if (!flow_vectors.empty()) {
        double weighted_sum_x = 0.0, weighted_sum_y = 0.0, weight_sum = 0.0;
        for (size_t i = 0; i < flow_vectors.size(); ++i) {
            weighted_sum_x += flow_vectors[i].x * weights[i];
            weighted_sum_y += flow_vectors[i].y * weights[i];
            weight_sum += weights[i];
        }
        flow_x = weight_sum > 0 ? weighted_sum_x / weight_sum : 0.0;
        flow_y = weight_sum > 0 ? weighted_sum_y / weight_sum : 0.0;
    }

    flow_x *= 2.0;
    flow_y *= 2.0;

    if (include_augmented_image)
        get_augmented_image(curr_image, good_new, good_old);
    
    return cv::Point2f(flow_x,flow_y);
}


// Метод для отрисовки направлений смещения особых точек - пока не сохраняем 
// это изображение в отдельный файл и не отображаем
void OpticalFlowLkt::get_augmented_image(const cv::Mat& curr_image, std::vector<cv::Point2f> good_new,
                                         std::vector<cv::Point2f> good_old){

    cv::Mat augmented_image = curr_image.clone();

    double scale_factor = 2.0;
    int h_orig = curr_image.rows;
    int w_orig = curr_image.cols;
    int h_crop = static_cast<int>(h_orig * 0.6);
    int w_crop = static_cast<int>(w_orig * 0.6);
    int offset_y = (h_orig - h_crop) / 2;
    int offset_x = (w_orig - w_crop) / 2;

    for (size_t i = 0; i < good_new.size(); ++i) {
        double a = good_new[i].x * scale_factor;
        double b = good_new[i].y * scale_factor;
        double c = good_old[i].x * scale_factor;
        double d = good_old[i].y * scale_factor;

        a += offset_x;
        b += offset_y;
        c += offset_x;
        d += offset_y;

        int ia = static_cast<int>(a);
        int ib = static_cast<int>(b);
        int ic = static_cast<int>(c);
        int id = static_cast<int>(d);

        if (ia >= 0 && ia < w_orig && ib >= 0 && ib < h_orig &&
            ic >= 0 && ic < w_orig && id >= 0 && id < h_orig) {
            cv::line(augmented_image, cv::Point(ia, ib), cv::Point(ic, id), cv::Scalar(0), 4);
            cv::circle(augmented_image, cv::Point(ia, ib), 7, cv::Scalar(0), -1);
        }
    }

    return;
}