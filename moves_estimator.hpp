#ifndef MOVES_ESTIMATOR_H_
#define MOVES_ESTIMATOR_H_

#include <opencv2/calib3d.hpp>

#include "image_data.hpp"

class MovesEstimator
{
public:
    explicit MovesEstimator(float angle = 0);
    cv::Mat EstimateMovements(ImageData const & next);
protected:
    void InitMatrix(float angle);
    void CosnstantZoom(cv::Mat& mat);
private:
    cv::Mat prev;
};

#endif // MOVES_ESTIMATOR_H_