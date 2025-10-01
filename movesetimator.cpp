#include "movesetimator.hpp"

using namespace std;
using namespace cv;

MoveEstimator::MoveEstimator(float angle)
{
    InitMatrix(angle);
}

cv::Mat MoveEstimator::EstimateMovements(ImageData const &next)
{
    auto keys1 = next.FirstKeypoints();
    auto keys2 = next.SecondKeypoints();
    vector<Point2f> first, second;
    auto matches = next.Matches();
    for (auto const & match : matches)
    {
        first.push_back(Point2f(keys1[match.queryIdx].pt));
        second.push_back(Point2f(keys2[match.trainIdx].pt));
    }
    
    Mat transform = estimateAffinePartial2D(first, second, noArray(), RANSAC, 3);
    CosnstantZoom(transform);
    invertAffineTransform(transform, transform);

    transform.push_back(Mat(vector<double>{0,0,1.0}).t());
    m_movemetns.push_back(transform);
    
    return transform;
}

void MoveEstimator::Reset(float angle)
{
    m_movemetns.clear();
    InitMatrix(angle);
}

void MoveEstimator::InitMatrix(float angle)
{
    Mat first = getRotationMatrix2D(Point2f(0,0), angle, 1);
    first.push_back(Mat(vector<double>{0,0,1.0}).t());
    m_movemetns.push_back(first);
}

void MoveEstimator::CosnstantZoom(Mat &mat)
{
	double* first  = mat.ptr<double>(0);
	double* second = mat.ptr<double>(1);
	double scale = sqrt(first[0]*second[1]-first[1]*second[0]);
	first[0] /= scale;
	first[1] /= scale;
	second[0] /= scale;
	second[1] /= scale;
}   
