#ifndef POINT
#define POINT

#include <dlib/matrix.h>

using namespace dlib;

class Point : public matrix<double, 4, 1> {

    public:

        Point() {

        }

        Point(matrix<double> m) {
            // TO DO: asserts
            double inv_m3 = 1.0 / m(3);

            this->operator ()(0) = m(0) * inv_m3;
            this->operator ()(1) = m(1) * inv_m3;
            this->operator ()(2) = m(2) * inv_m3;
            this->operator ()(3) = 1.0;
        }

        Point(matrix<double, 1, 4> &m) {

            double inv_m3 = 1.0 / m(3);

            this->operator ()(0) = m(0) * inv_m3;
            this->operator ()(1) = m(1) * inv_m3;
            this->operator ()(2) = m(2) * inv_m3;
            this->operator ()(3) = 1.0;

        }

        Point(double x, double y, double z) {
            this->operator ()(0) = x;
            this->operator ()(1) = y;
            this->operator ()(2) = z;
            this->operator ()(3) = 1.0;
        }

        matrix<double, 4, 1> innerMatrix() {
            matrix<double, 4, 1> p;
            p(0) = this->operator ()(0);
            p(1) = this->operator ()(1);
            p(2) = this->operator ()(2);
            p(3) = this->operator ()(3);
            return p;
        }

};

#endif // POINT

