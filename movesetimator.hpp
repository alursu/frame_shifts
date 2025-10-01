#pragma once

#include <opencv2/calib3d.hpp>

#include "imagedata.hpp"

class MoveEstimator
{
public:
    explicit MoveEstimator(float angle = 0);
    cv::Mat EstimateMovements(ImageData const & next);
    std::vector<cv::Mat> GetMovements() const {return m_movemetns;}
    void Reset(float angle = 0);
protected:
    void InitMatrix(float angle);
    void CosnstantZoom(cv::Mat& mat);
    std::vector<cv::Mat> m_movemetns;
};