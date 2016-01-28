#include <dlib/matrix.h>
#include <limits>

using namespace std;
using namespace dlib;

enum TYPE_NULL_SPACE_VECTOR {
    LEFT_NULL_SPACE_VECTOR,
    RIGHT_NULL_SPACE_VECTOR
};


matrix<double> getNullSpaceVector(const matrix<double> &m, TYPE_NULL_SPACE_VECTOR type) {
    matrix<double> u, s, vt;
    matrix<double> null_space_vector;
    svd(m, u, s, vt);
    double min_autovalue = numeric_limits<double>::max();
    int min_index = -1;
    for (int i = 0; i < s.nr(); i++) {
        if (s(i, i) < min_autovalue) {
            min_autovalue = s(i, i);
            min_index = i;
        }
    }
    if (type == RIGHT_NULL_SPACE_VECTOR) {
        null_space_vector = colm(vt, min_index);
    } else {
        null_space_vector = colm(u, min_index);
    }
    return null_space_vector;
}
