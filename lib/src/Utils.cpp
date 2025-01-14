#include "Utils.h"

Eigen::Matrix3f skew(Vector3f v)
{
    Eigen::Matrix3f skew_matrix;
    skew_matrix << 0, -v(2), v(1),
                   v(2), 0, -v(0),
                   -v(1), v(0), 0;
    return skew_matrix;
}