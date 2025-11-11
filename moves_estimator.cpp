#include "moves_estimator.hpp"

using namespace std;
using namespace cv;

MovesEstimator::MovesEstimator(float angle)
{
    // Инициализация матрицы афинных преобразований 1 кадра
    // Задаем угол поворота 1 кадра относительно желаемой СК
    init_matrix(angle);
}

cv::Mat MovesEstimator::estimate_movements(ImageData const &next)
{
    auto keys1 = next.first_keypoints();
    auto keys2 = next.second_keypoints();
    vector<Point2f> first, second;
    auto matches = next.matches();
    for (auto const & match : matches)
    {
        // записываем координаты соответствующих друг другу особых
        // точек двух соседних кадров, т. е. какие координаты имеет
        // одна и та же точка на разных изображениях
        first.push_back(Point2f(keys1[match.queryIdx].pt));
        second.push_back(Point2f(keys2[match.trainIdx].pt));
    }
    
    // вычисляем по координатам матрицу афинных преобразований
    // в матрице содержатся угол поворота, смещение и коэффициент масштабирования
    // все значения определены в СК нынешнего кадра.
    // Ориентация осей : y - вниз; x - вправо.
    // Угол поворота считается "+", если вращение по часовой стрелке.
    Mat transform = estimateAffinePartial2D(first, second, noArray(), RANSAC, 3);

    // Убираем коэффициент масштабирования из матрицы
    cosnstant_zoom(transform);
    
    // Инвертируем матрицу, чтобы получить значения в СК предыдущего кадра
    invertAffineTransform(transform, transform);

    transform.push_back(Mat(vector<double>{0,0,1.0}).t());

    // Домножаем на прошлую матрицу, чтобы все значения смещений 
    // были в единой СК - СК 1-го кадра
    transform = prev * transform;
    prev = transform;
    return transform;
}

void MovesEstimator::init_matrix(float angle)
{
    // Единичная матрица (для угла = 0 и центральной точки = (0,0))
    prev = getRotationMatrix2D(Point2f(0,0), angle, 1);
    prev.push_back(Mat(vector<double>{0,0,1.0}).t());
}

void MovesEstimator::cosnstant_zoom(Mat &mat)
{
    // Вычисляем коэффициент масштабирования и убираем его из матрицы
    //        Изначальная матрица афинных преобразований
    //               s*cosx       -s*sinx     tx
    //               s*sinx        s*cosx     ty
    //                 0             0         1
	double* first  = mat.ptr<double>(0);
	double* second = mat.ptr<double>(1);
    //              s = sqrt(s^2*cosx^2+s^2sinx^2)
	double scale = sqrt(first[0]*second[1]-first[1]*second[0]);
	first[0] /= scale;
	first[1] /= scale;
	second[0] /= scale;
	second[1] /= scale;
    //                Матрица после деления
    //               cosx       -sinx     tx
    //               sinx        cosx     ty
    //                0            0       1
    //
    //     Где x - угол поворота; tx, ty - смещения
}   
