//внедрить это в пайплайн и пусть prev_image отправляется
#include "opticalflowlkt.hpp"

bool REV_FLOW = false;


//почитстить от ненужных логов, они не нужны, как и вычисления времени
// убрать ненужные и неиспользуемые проверки
std::map<std::string, bool> get_optical_flow(const cv::Mat& curr_image, double capture_time, bool include_augmented_image) {
    std::string logging_prefix_str = "get_optical_flow:";

    clock_t start_time = clock();

    // переписать вывод
    if (curr_image.empty()) {
        std::cerr << logging_prefix_str << " Current image is empty" << std::endl;
        return { {"success", false} };
    }

    // убрать копирование в curr_image_grey и проверку на трехканальность? или так оптимальнее?
    cv::Mat curr_image_grey;
    if (curr_image.channels() == 3) {
        cv::cvtColor(curr_image, curr_image, cv::COLOR_BGR2GRAY);
    } else {
        curr_image_grey = curr_image;
    }

    // Оптимизация - используем только центральные 60% изображения для обработки
    // Мб использовать и для BRISK? Но с меньшим процентом. Мб переписать без этой кучи переменной
    int h = curr_image_grey.rows;
    int w = curr_image_grey.cols;
    double crop_factor = 0.6;
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
        // зачем нужен этот таймер? удалить?
        prev_image_time = capture_time;

        std::cerr << logging_prefix_str << " No previous image available for optical flow calculation" << std::endl;
        return { {"success", false} };
    }

    // Parameters for Shi-Tomasi corner detection
    std::vector<cv::Point2f> corners0;
    int maxCorners = 50;
    double qualityLevel = 0.2;
    double minDistance = 10;
    int blockSize = 5;

    cv::goodFeaturesToTrack(prev_image, corners0, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize);

    if (corners0.empty()) {
        // зачем замерять это время?
        double dt = capture_time - prev_image_time;
        prev_image = curr_image_grey.clone();
        prev_image_time = capture_time;
        std::cerr << logging_prefix_str << " No corners detected in the previous image" << std::endl;
        return { {"success", false} };
    }

    // Parameters for Lucas-Kanade optical flow
    cv::Size winSize(15, 15);
    int maxLevel = 2;
    cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 20, 0.03);

    std::vector<cv::Point2f> corners1;
    std::vector<uchar> st;
    std::vector<float> err;

    cv::calcOpticalFlowPyrLK(prev_image, curr_image_grey, corners0, corners1, st, err, winSize, maxLevel, criteria);

    int good_count = 0;
    for (auto s : st) good_count += s;

    if (good_count < 5) {
        // Retry with smaller window and level
        cv::Size retryWinSize(12, 12);
        int retryMaxLevel = 1;
        cv::calcOpticalFlowPyrLK(prev_image, curr_image_grey, corners0, corners1, st, err, retryWinSize, retryMaxLevel);
    }

    // зачем нужен rev_flow??? - проверка обратным преобразоваинем
    // по сути делаем то же, но наоборот и сравниваем полученные значения
    // должен быть выставлен флаг успешного определения значений в обоих случаях
    // ну и расстояние между изначальными точками и посчитанными (точки и previous_image) должно быть меньше 0.5
    // по сути не используется, сторонне флаг не устанавливается
    if (REV_FLOW) {
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

    double dt = capture_time - prev_image_time;
    prev_image = curr_image_grey.clone();
    prev_image_time = capture_time;

    // Filter good points
    std::vector<cv::Point2f> good_new, good_old;
    std::vector<float> good_errors;
    for (size_t i = 0; i < st.size(); ++i) {
        if (st[i]) {
            good_new.push_back(corners1[i]);
            good_old.push_back(corners0[i]);
            good_errors.push_back(err[i]);
        }
    }

    // какая-то странная проверка, непонятно для чего нужна, если есть косяк, то он всплывет в for (цикл выше), ну кроме проверки, пустой ли вектор
    if (good_new.size() != good_old.size() || good_new.empty()) {
        std::cerr << logging_prefix_str << " Invalid points array shapes, new:" << good_new.size()
                  << ", old:" << good_old.size() << ", errors:" << good_errors.size() << std::endl;
        return { {"success", false} };
    }

    // Compute flow vectors
    std::vector<cv::Point2f> flow_vectors;
    for (size_t i = 0; i < good_new.size(); ++i) {
        flow_vectors.push_back(good_new[i] - good_old[i]);
    }

    std::cerr << logging_prefix_str << " good_new size: " << good_new.size()
              << ", good_old size: " << good_old.size()
              << ", good_errors size: " << good_errors.size()
              << ", flow_vectors size: " << flow_vectors.size() << std::endl;

    return { {"success", true} };
}

std::map<std::string, bool> processOpticalFlow(
    const std::vector<cv::Point2f>& flow_vectors,
    const cv::Mat& good_errors,
    const std::vector<cv::Point2f>& good_new,
    const std::vector<cv::Point2f>& good_old,
    const cv::Mat& curr_image,
    bool include_augmented_image,
    double dt,
    const std::string& logging_prefix_str,
    std::chrono::steady_clock::time_point start_time,
    double corner_time,
    double lk_time,
    std::ostream& logger // Simplified logger as ostream
) {
    try {
        if (flow_vectors.size() == 0) {
            logger << logging_prefix_str << " No points successfully tracked" << std::endl;
            return { {"success", false} };
        }

        // Calculate weights = 1.0 / (good_errors + 1e-6)
        std::vector<double> weights;
        weights.reserve(good_errors.total());
        for (int i = 0; i < good_errors.total(); ++i) {
            double val = good_errors.at<float>(i);
            weights.push_back(1.0 / (val + 1e-6));
        }

        // Weighted average for flow_x and flow_y
        double flow_x = 0.0, flow_y = 0.0;
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

        // Convert to float (double in C++ is fine)
        float flow_x_f = static_cast<float>(flow_x);
        float flow_y_f = static_cast<float>(flow_y);
        float dt_f = static_cast<float>(dt);

        auto total_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();

        logger << logging_prefix_str << " INTERNAL_TIMING: Total:" << std::fixed << std::setprecision(1) << total_time * 1000 << "ms "
               << "CornerDetect:" << corner_time * 1000 << "ms LK:" << lk_time * 1000 << "ms "
               << "Features:" << good_new.size() << std::endl;

        std::string image_base64;
        if (include_augmented_image) {
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
                    cv::line(augmented_image, cv::Point(ia, ib), cv::Point(ic, id), cv::Scalar(0, 255, 0), 2);
                    cv::circle(augmented_image, cv::Point(ia, ib), 5, cv::Scalar(0, 0, 255), -1);
                }
            }

            std::vector<uchar> buffer;
            cv::imencode(".jpg", augmented_image, buffer);

            // Base64 encode
            image_base64 = base64_encode(buffer.data(), buffer.size());
        }

        return { {"success", true} };
    }
    catch (const std::exception& e) {
        logger << "Error during Optical Flow calculation: " << e.what() << std::endl;
        return { {"success", false} };
    }
}