#pragma once

#include <vector>
#include <cmath>
#include <iostream>

namespace poly_lib {

class Poly1D {
public:
    enum BasisType : uint8_t {
        STANDARD = 0,
        CHEBYSHEV,
        LEGENDRE,
        HERMITE,
        LAGUERRE
    };

    Poly1D() = default;

    Poly1D(const std::vector<float>& coefs, BasisType basis)
        : coefficients_(coefs), basis_type_(basis) {}

    void setCoefficients(const std::vector<float>& coefs) {
        coefficients_ = coefs;
    }

    void setBasisType(BasisType basis) {
        basis_type_ = basis;
    }

    double evaluate(double t) const {
        switch (basis_type_) {
            case STANDARD:  return evalStandard(t);
            case CHEBYSHEV: return evalChebyshev(t);
            case LEGENDRE:  return evalLegendre(t);
            case HERMITE:   return evalHermite(t);
            case LAGUERRE:  return evalLaguerre(t);
            default:
                std::cerr << "Unknown basis type: " << static_cast<int>(basis_type_) << std::endl;
                return 0.0;
        }
    }

private:
    std::vector<float> coefficients_;
    BasisType basis_type_ = STANDARD;

    double evalStandard(double t) const {
        double result = 0.0;
        for (size_t i = 0; i < coefficients_.size(); ++i) {
            result += coefficients_[i] * std::pow(t, static_cast<int>(i));
        }
        return result;
    }

    double evalChebyshev(double t) const {
        if (coefficients_.empty()) return 0.0;
        std::vector<double> T(coefficients_.size());
        T[0] = 1.0;
        if (coefficients_.size() == 1)
            return coefficients_[0] * T[0];
        T[1] = t;
        double result = coefficients_[0] * T[0] + coefficients_[1] * T[1];
        for (size_t n = 2; n < coefficients_.size(); ++n) {
            T[n] = 2 * t * T[n - 1] - T[n - 2];
            result += coefficients_[n] * T[n];
        }
        return result;
    }

    double evalLegendre(double t) const {
        if (coefficients_.empty()) return 0.0;
        std::vector<double> P(coefficients_.size());
        P[0] = 1.0;
        if (coefficients_.size() == 1)
            return coefficients_[0] * P[0];
        P[1] = t;
        double result = coefficients_[0] * P[0] + coefficients_[1] * P[1];
        for (size_t n = 2; n < coefficients_.size(); ++n) {
            P[n] = ((2 * n - 1) * t * P[n - 1] - (n - 1) * P[n - 2]) / n;
            result += coefficients_[n] * P[n];
        }
        return result;
    }

    double evalHermite(double t) const {
        if (coefficients_.empty()) return 0.0;
        std::vector<double> H(coefficients_.size());
        H[0] = 1.0;
        if (coefficients_.size() == 1)
            return coefficients_[0] * H[0];
        H[1] = 2.0 * t;
        double result = coefficients_[0] * H[0] + coefficients_[1] * H[1];
        for (size_t n = 2; n < coefficients_.size(); ++n) {
            H[n] = 2.0 * t * H[n - 1] - 2.0 * (n - 1) * H[n - 2];
            result += coefficients_[n] * H[n];
        }
        return result;
    }

    double evalLaguerre(double t) const {
        if (coefficients_.empty()) return 0.0;
        std::vector<double> L(coefficients_.size());
        L[0] = 1.0;
        if (coefficients_.size() == 1)
            return coefficients_[0] * L[0];
        L[1] = 1.0 - t;
        double result = coefficients_[0] * L[0] + coefficients_[1] * L[1];
        for (size_t n = 2; n < coefficients_.size(); ++n) {
            L[n] = ((2 * n - 1 - t) * L[n - 1] - (n - 1) * L[n - 2]) / n;
            result += coefficients_[n] * L[n];
        }
        return result;
    }
};

}  // namespace poly_lib

