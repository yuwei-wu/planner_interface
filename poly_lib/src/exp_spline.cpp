#include <iostream>
#include "poly_lib/bspline.hpp"

using namespace poly_lib;

int main() {
    // 10 control points in 3D (rows = 10, cols = 3)
    Eigen::MatrixXd cpts(10, 3);
    cpts << 0, 0, 0,
            1, 2, 1,
            2, 1, 2,
            3, 3, 3,
            4, 2, 1,
            5, 0, 2,
            6, 1, 0,
            7, 3, 1,
            8, 2, 2,
            9, 0, 0;

    // Cubic B-spline (degree=3) with Bezier basis (BERNSTEIN enum value)
    Bspline bspline(3, Bspline::BERNSTEIN);

    bspline.setControlPoints(cpts);

    // Open uniform knot vector (size = num_ctrl_pts + degree + 1 = 14)
    std::vector<double> knots = {
        0, 0, 0, 0,   // degree+1 repeated start
        1, 2, 3, 4, 5, 6,
        7, 7, 7, 7    // degree+1 repeated end
    };

    bspline.setKnots(knots);

    // Convert to PolyND segments (vector of PolyND)
    std::vector<PolyND> polys = bspline.toPolyNDs();

    // Evaluate the first polynomial segment at normalized parameter t=0.5
    double t = 0.5;
    if (!polys.empty()) {
        Eigen::VectorXd val = polys[0].evaluate(t);
        std::cout << "Value of first poly segment at t=0.5: " << val.transpose() << std::endl;
    } else {
        std::cerr << "No polynomial segments generated." << std::endl;
    }

    return 0;
}
