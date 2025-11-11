#ifndef MOVES_ESTIMATOR_H_
#define MOVES_ESTIMATOR_H_

#include <opencv2/calib3d.hpp>

#include "image_data.hpp"

class MovesEstimator
{
public:

    explicit MovesEstimator(float angle = 0);
    
    cv::Mat estimate_movements(ImageData const & next);

private:

    void init_matrix(float angle);
    void cosnstant_zoom(cv::Mat& mat);
    cv::Mat prev;
};

#endif // MOVES_ESTIMATOR_H_