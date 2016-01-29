#include <iostream>
#include <stitcher.h>

using namespace std;

int main() {

    dlib::matrix<double> imageA;
    dlib::matrix<double> imageB;

    read_depth_image("samples/rgbd_01330.depth", "samples/rgbd_01301.depth", imageA, imageB);

    std::vector<Point> P;
    std::vector<Point> P_;

    P.push_back(Point(175, 213, imageA(213, 175)));
    P_.push_back(Point(48, 229, imageB(229, 48)));

    P.push_back(Point(451, 386, imageA(386, 451)));
    P_.push_back(Point(351, 357, imageB(357, 351)));

    P.push_back(Point(209, 166, imageA(166, 209)));
    P_.push_back(Point(78, 175, imageB(175, 78)));

    P.push_back(Point(180, 311, imageA(311, 180)));
    P_.push_back(Point(69, 332, imageB(332, 69)));

//    P.push_back(Point(323, 461, imageA(461, 323)));
//    P_.push_back(Point(240, 452, imageB(452, 240)));

    matrix<double, 4, 4> H;
    get_homography(P, P_, H);

//    H = trans(H);

    for (int i = 0; i < P.size(); i++) {
        matrix<double, 4, 1> K = inv(H) * P[i];
        K /= K(3);
        cout << trans(P_[i]) << " >> " << trans(K) << endl;
    }

    cout << H << endl;

    return 0;
}

