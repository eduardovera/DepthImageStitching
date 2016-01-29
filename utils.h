#include <dlib/matrix.h>
#include <limits>
#include <rgbdio.h>

using namespace std;
using namespace dlib;

enum TYPE_NULL_SPACE_VECTOR {
    LEFT_NULL_SPACE_VECTOR,
    RIGHT_NULL_SPACE_VECTOR
};


matrix<double> getNullSpaceVector(const matrix<double> &m, TYPE_NULL_SPACE_VECTOR type) {
    matrix<double> u, s, v;
    matrix<double> null_space_vector;
    svd(m, u, s, v);
    double min_autovalue = numeric_limits<double>::max();
    int min_index = -1;
    for (int i = 0; i < s.nr(); i++) {
        if (s(i, i) < min_autovalue) {
            min_autovalue = s(i, i);
            min_index = i;
        }
    }
    if (type == RIGHT_NULL_SPACE_VECTOR) {
        null_space_vector = colm(v, min_index);
    } else {
        null_space_vector = colm(u, min_index);
    }
    return null_space_vector;
}


void read_depth_image(string filenameA, string filenameB, matrix<double> &imageA, matrix<double> &imageB) {
    rgbd::DepthFrame frameA;
    rgbd::DepthFrame frameB;

    frameA.load(filenameA);
    frameB.load(filenameB);

    assert(frameA.width() == frameB.width());
    assert(frameA.height() == frameB.height());

    imageA.set_size(frameA.height(), frameA.width());
    imageB.set_size(frameB.height(), frameB.width());

    for (int j = 0, index = 0; j < frameA.height(); j++) {
        for (int i = 0; i < frameA.width(); i++, index++) {
            imageA(j, i) = frameA.data()[index];
            imageB(j, i) = frameB.data()[index];

//            cout << imageA(j, i) << endl;
        }
    }
}
