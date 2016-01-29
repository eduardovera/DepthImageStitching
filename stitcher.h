#include <iostream>
#include <point.h>
#include <limits>
#include <utils.h>

using namespace std;
using namespace dlib;

void get_homography(std::vector<Point> &P, std::vector<Point> &P_, matrix<double, 4, 4> &H) {
    matrix<double, 24, 16> A;
    A = zeros_matrix(A);

    for (int i = 0; i < A.nr(); i++) {
        int mod = i % 6;
        int index = i / 6;
        if (mod == 0) {
            A(i, 0) = -P_[index](1) * P[index](0);
            A(i, 1) = -P_[index](1) * P[index](1);
            A(i, 2) = -P_[index](1) * P[index](2);
            A(i, 3) = -P_[index](1);
            A(i, 4) = P_[index](0) * P[index](0);
            A(i, 5) = P_[index](0) * P[index](1);
            A(i, 6) = P_[index](0) * P[index](2);
            A(i, 7) = P_[index](0);
        } else if (mod == 1) {
            A(i, 0) = -P_[index](2) * P[index](0);
            A(i, 1) = -P_[index](2) * P[index](1);
            A(i, 2) = -P_[index](2) * P[index](2);
            A(i, 3) = -P_[index](2);
            A(i, 8) = P_[index](0) * P[index](0);
            A(i, 9) = P_[index](0) * P[index](1);
            A(i, 10) = P_[index](0) * P[index](2);
            A(i, 11) = P_[index](0);
        } else if (mod == 2) {
            A(i, 4) = -P_[index](2) * P[index](0);
            A(i, 5) = -P_[index](2) * P[index](1);
            A(i, 6) = -P_[index](2) * P[index](2);
            A(i, 7) = -P_[index](2);
            A(i, 8) = P_[index](0) * P[index](0);
            A(i, 9) = P_[index](0) * P[index](1);
            A(i, 10) = P_[index](0) * P[index](2);
            A(i, 11) = P_[index](0);
        } else if (mod == 3) {
            A(i, 0) = -P[index](0);
            A(i, 1) = -P[index](1);
            A(i, 2) = -P[index](2);
            A(i, 3) = -1;
            A(i, 12) = P_[index](0) * P[index](0);
            A(i, 13) = P_[index](0) * P[index](1);
            A(i, 14) = P_[index](0) * P[index](2);
            A(i, 15) = P_[index](0);
        } else if (mod == 4) {
            A(i, 4) = -P[index](0);
            A(i, 5) = -P[index](1);
            A(i, 6) = -P[index](2);
            A(i, 7) = -1;
            A(i, 12) = P_[index](1) * P[index](0);
            A(i, 13) = P_[index](1) * P[index](1);
            A(i, 14) = P_[index](1) * P[index](2);
            A(i, 15) = P_[index](1);
        } else if (mod == 5) {
            A(i, 8) = -P[index](0);
            A(i, 9) = -P[index](1);
            A(i, 10) = -P[index](2);
            A(i, 11) = -1;
            A(i, 12) = P_[index](2) * P[index](0);
            A(i, 13) = P_[index](2) * P[index](1);
            A(i, 14) = P_[index](2) * P[index](2);
            A(i, 15) = P_[index](2);
        }
    }

    H = getNullSpaceVector(A, RIGHT_NULL_SPACE_VECTOR);
    H = reshape(H, 4, 4);


}

