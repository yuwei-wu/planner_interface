
#pragma once

#include <vector>
#include <Eigen/Eigen>
#include <stdexcept>
#include "poly_nd.hpp"

namespace poly_lib {

class Bspline {
public:
    enum BasisType : uint8_t {
        BEZIER = 0,
        BERNSTEIN = 1,
        STANDARD = 2  // Added for clarity in PolyND creation
    };

    Bspline(int degree, BasisType basis_type)
        : degree_(degree), basis_type_(basis_type)
    {
        if (degree_ < 1 || degree_ > 4) {
            throw std::runtime_error("Degree must be between 1 and 4.");
        }
    }

    void setControlPoints(const Eigen::MatrixXd& cpts) {
        if (cpts.rows() == 0 || cpts.cols() == 0) {
            throw std::runtime_error("Control points must be non-empty.");
        }
        control_points_ = cpts;
    }

    void setKnots(const std::vector<double>& knots) {
        if (knots.size() < control_points_.rows() + degree_ + 1) {
            throw std::runtime_error("Knot vector size inconsistent with control points and degree.");
        }
        knots_ = knots;
    }

    double basisFunction(int i, int p, double t) const {
        if (p == 0) {
            if (knots_[i] <= t && t < knots_[i + 1]) {
                return 1.0;
            }
            if (t == knots_.back() && i == (int)knots_.size() - p - 2) {
                return 1.0;
            }
            return 0.0;
        }

        double denom1 = knots_[i + p] - knots_[i];
        double denom2 = knots_[i + p + 1] - knots_[i + 1];
        double term1 = 0.0, term2 = 0.0;

        if (denom1 > 1e-12) {
            term1 = (t - knots_[i]) / denom1 * basisFunction(i, p - 1, t);
        }
        if (denom2 > 1e-12) {
            term2 = (knots_[i + p + 1] - t) / denom2 * basisFunction(i + 1, p - 1, t);
        }

        return term1 + term2;
    }

    Eigen::VectorXd evaluate(double t) const {
        if (control_points_.rows() == 0) {
            throw std::runtime_error("Control points not set.");
        }
        if (knots_.empty()) {
            throw std::runtime_error("Knot vector not set.");
        }

        int n = (int)control_points_.rows() - 1;
        int p = degree_;
        Eigen::VectorXd result = Eigen::VectorXd::Zero(control_points_.cols());

        for (int i = 0; i <= n; ++i) {
            double b = basisFunction(i, p, t);
            result += b * control_points_.row(i).transpose();
        }
        return result;
    }

    std::vector<PolyND> toPolyNDs() const {
        if (knots_.empty()) {
            throw std::runtime_error("Knot vector must be set.");
        }

        int p = degree_;
        int dim = (int)control_points_.cols();
        int num_segments = (int)knots_.size() - 1 - 2 * p;

        if (num_segments <= 0) {
            throw std::runtime_error("Invalid number of segments.");
        }

        std::vector<PolyND> polys;
        polys.reserve(num_segments);

        for (int seg = 0; seg < num_segments; ++seg) {
            double t0 = knots_[p + seg];
            double t1 = knots_[p + seg + 1];

            int sample_num = p + 1;
            Eigen::MatrixXd samples(dim, sample_num);
            Eigen::VectorXd ts(sample_num);

            for (int s = 0; s < sample_num; ++s) {
                double t = t0 + (t1 - t0) * s / (sample_num - 1);
                ts(s) = t;
                samples.col(s) = evaluate(t);
            }

            Eigen::MatrixXd V(sample_num, p + 1);
            for (int i = 0; i < sample_num; ++i) {
                double val = 1.0;
                double normalized_t = (ts(i) - t0) / (t1 - t0);
                for (int j = 0; j <= p; ++j) {
                    V(i, j) = val;
                    val *= normalized_t;
                }
            }

            std::vector<std::vector<float>> poly_coefs(dim, std::vector<float>(p + 1));
            for (int d = 0; d < dim; ++d) {
                Eigen::VectorXd y = samples.row(d).transpose();
                Eigen::VectorXd c = V.colPivHouseholderQr().solve(y);
                for (int j = 0; j <= p; ++j) {
                    poly_coefs[d][j] = static_cast<float>(c(j));
                }
            }

            PolyND poly(dim, p, 1, Poly1D::STANDARD);  // Use Poly1D::STANDARD basis
            poly.setSegCoefficients(0, poly_coefs);
            polys.push_back(poly);
        }

        return polys;
    }

private:
    int degree_;
    BasisType basis_type_;
    Eigen::MatrixXd control_points_;
    std::vector<double> knots_;
};

}  // namespace poly_lib