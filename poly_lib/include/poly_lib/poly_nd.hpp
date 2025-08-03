#pragma once

#include <vector>
#include <Eigen/Dense>
#include <stdexcept>
#include "poly_1d.hpp"  // Poly1D class

namespace poly_lib {

class PolyND {
public:
    // Using Poly1D::BasisType directly for basis
    PolyND(int dimensions, int order, int num_segments, Poly1D::BasisType basis_type)
        : N_(dimensions), order_(order), M_(num_segments), basis_type_(basis_type)
    {
        segments_.resize(M_, std::vector<Poly1D>(N_));
        segment_durations_.resize(M_, 1.0);
    }

    void setSegDurations(const std::vector<double>& durations) {
        if (durations.size() != M_) {
            throw std::runtime_error("Segment durations size mismatch.");
        }
        segment_durations_ = durations;
    }

    // Correct method name to set coefficients for a specific segment
    void setSegCoefficients(int segment_idx, const std::vector<std::vector<float>>& coefs) {
        if (segment_idx < 0 || segment_idx >= M_) {
            throw std::runtime_error("Invalid segment index.");
        }
        if (coefs.size() != N_) {
            throw std::runtime_error("Coefficient dimensions mismatch.");
        }
        for (int d = 0; d < N_; ++d) {
            segments_[segment_idx][d] = Poly1D(coefs[d], basis_type_);
        }
    }

    // Evaluate multi-dimensional polynomial at global parameter t_global
    Eigen::VectorXd evaluate(double t_global) const {
        int seg_idx = findSegment(t_global);
        double t_local = localTime(t_global, seg_idx);

        Eigen::VectorXd result(N_);
        for (int d = 0; d < N_; ++d) {
            result(d) = segments_[seg_idx][d].evaluate(t_local);
        }
        return result;
    }

    Eigen::VectorXd derivative(double t_global, int der_order) const {
        double dt = 1e-5;
        Eigen::VectorXd val_plus = evaluate(t_global + dt);
        Eigen::VectorXd val_minus = evaluate(t_global - dt);
        return (val_plus - val_minus) / (2.0 * dt);
    }

private:
    int N_;  // Dimensions
    int order_;
    int M_;  // Number of segments
    Poly1D::BasisType basis_type_;

    std::vector<double> segment_durations_;
    std::vector<std::vector<Poly1D>> segments_;

    int findSegment(double t_global) const {
        double cum_time = 0.0;
        for (int i = 0; i < M_; ++i) {
            cum_time += segment_durations_[i];
            if (t_global <= cum_time) return i;
        }
        return M_ - 1;
    }

    double localTime(double t_global, int segment_idx) const {
        double cum_time = 0.0;
        for (int i = 0; i < segment_idx; ++i) {
            cum_time += segment_durations_[i];
        }
        return t_global - cum_time;
    }
};

}