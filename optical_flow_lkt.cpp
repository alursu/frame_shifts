#include "optical_flow_lkt.hpp"

cv::Point2f OpticalFlowLkt::GetOpticalFlow(const cv::Mat& curr_image, bool include_augmented_image, bool rev_flow) {

    if (curr_image.empty()) {
        return cv::Point2f(0,0);
         //что-то нужно с этим сделать, как-то обработать
    }   

    cv::Mat curr_image_grey = curr_image.clone();

    // Оптимизация - используем только центральные 60% изображения для обработки
    // Мб использовать и для BRISK? Но с меньшим процентом. Мб переписать без этой кучи переменных
    int h = curr_image_grey.rows;
    int w = curr_image_grey.cols;
    int crop_h = static_cast<int>(h * crop_factor);
    int crop_w = static_cast<int>(w * crop_factor);
    int start_y = (h - crop_h) / 2;
    int start_x = (w - crop_w) / 2;
    cv::Rect roi(start_x, start_y, crop_w, crop_h);
    curr_image_grey = curr_image_grey(roi);

    // Оптимизация со снижением разрешения в 2 раза? Реально ли так отработает?
    cv::resize(curr_image_grey, curr_image_grey, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);

    if (prev_image.empty()) {
        prev_image = curr_image_grey.clone();
        return cv::Point2f(0,0);
        // надо обработать
    }

    std::vector<cv::Point2f> corners0;

    cv::goodFeaturesToTrack(prev_image, corners0, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize);

    if (corners0.empty()) {
        prev_image = curr_image_grey.clone();
        return cv::Point2f(0,0);
        // надо обработать
    }

    std::vector<cv::Point2f> corners1;
    std::vector<uchar> st;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK(prev_image, curr_image_grey, corners0, corners1, st, err, winSize, maxLevel, criteria);

    int good_count = 0;
    for (auto s : st) good_count += s;

    if (good_count < 5) {
        cv::Size retryWinSize(12, 12);
        int retryMaxLevel = 1;
        cv::calcOpticalFlowPyrLK(prev_image, curr_image_grey, corners0, corners1, st, err, retryWinSize, retryMaxLevel);
    }

    if (rev_flow) {
        std::vector<cv::Point2f> rev_corners1;
        std::vector<uchar> stRev;
        std::vector<float> errRev;
        cv::calcOpticalFlowPyrLK(curr_image_grey, prev_image, corners1, rev_corners1, stRev, errRev, cv::Size(12, 12), 1, criteria);

        for (size_t i = 0; i < corners0.size(); ++i) {
            if (i >= rev_corners1.size() || i >= stRev.size()) break;
            double dist = cv::norm(corners0[i] - rev_corners1[i]);
            st[i] = st[i] & stRev[i] & (dist <= 0.5);
        }
    }

    prev_image = curr_image_grey.clone();

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

    // Calculate weights = 1.0 / (good_errors + 1e-6)
    std::vector<double> weights;
    weights.reserve(good_errors.size());
    for (int i = 0; i < good_errors.size(); ++i) {
        double val = good_errors[i];
        weights.push_back(1.0 / (val + 1e-6));
    }

    // Weighted average for flow_x and flow_y
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

    // std::string image_base64;
    // if (include_augmented_image)
    //     GetAugmentedImage(image_base64);
    
    return cv::Point2f(flow_x,flow_y);
}

// странная функция, которая по сути должна отображать результаты
// тут еще стоит переписать и покопаться
// std::string OpticalFlowLkt::GetAugmentedImage(std::string& image_base64){

    // cv::Mat augmented_image = curr_image.clone();

    // double scale_factor = 2.0;
    // int h_orig = curr_image.rows;
    // int w_orig = curr_image.cols;
    // int h_crop = static_cast<int>(h_orig * 0.6);
    // int w_crop = static_cast<int>(w_orig * 0.6);
    // int offset_y = (h_orig - h_crop) / 2;
    // int offset_x = (w_orig - w_crop) / 2;

    // for (size_t i = 0; i < good_new.size(); ++i) {
    //     double a = good_new[i].x * scale_factor;
    //     double b = good_new[i].y * scale_factor;
    //     double c = good_old[i].x * scale_factor;
    //     double d = good_old[i].y * scale_factor;

    //     a += offset_x;
    //     b += offset_y;
    //     c += offset_x;
    //     d += offset_y;

    //     int ia = static_cast<int>(a);
    //     int ib = static_cast<int>(b);
    //     int ic = static_cast<int>(c);
    //     int id = static_cast<int>(d);

    //     if (ia >= 0 && ia < w_orig && ib >= 0 && ib < h_orig &&
    //         ic >= 0 && ic < w_orig && id >= 0 && id < h_orig) {
    //         cv::line(augmented_image, cv::Point(ia, ib), cv::Point(ic, id), cv::Scalar(0, 255, 0), 2);
    //         cv::circle(augmented_image, cv::Point(ia, ib), 5, cv::Scalar(0, 0, 255), -1);
    //     }
    // }

    // std::vector<uchar> buffer;
    // cv::imencode(".jpg", augmented_image, buffer);

    // // Base64 encode
    // image_base64 = base64_encode(buffer.data(), buffer.size());
//     return "nothing yet";
// }