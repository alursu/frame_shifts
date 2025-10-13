#pragma once

#include <opencv2/calib3d.hpp>

#include "imagedata.hpp"

class MoveEstimator
{
public:
    explicit MoveEstimator(float angle = 0);
    cv::Mat EstimateMovements(ImageData const & next);
protected:
    void InitMatrix(float angle);
    void CosnstantZoom(cv::Mat& mat);
private:
    cv::Mat prev;
};