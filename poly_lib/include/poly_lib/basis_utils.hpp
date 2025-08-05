#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <vector>

/*
 * "Standard Basis":
 * 
 * The standard basis refers to the polynomial basis consisting of
 * powers of t: {1, t, t^2, t^3, ..., t^n}.
 * 
 * A polynomial expressed in the standard basis has the form:
 *    p(t) = c_0 + c_1 * t + c_2 * t^2 + ... + c_n * t^n,
 * where {c_0, c_1, ..., c_n} are the standard polynomial coefficients.
 * 
 * Orthogonal polynomial bases, such as Legendre or Chebyshev polynomials,
 * represent polynomials as sums of special basis functions {P_0(t), P_1(t), ...}.
 * 
 * Converting from Legendre (or Chebyshev) coefficients to standard coefficients
 * means rewriting the polynomial using powers of t instead of those special polynomials.
 */

// Recursive function to compute Chebyshev polynomial T_n(t) in standard basis
Eigen::VectorXd chebyshevToStandardBasis(int n) {
    if (n == 0) {
        Eigen::VectorXd T(1);
        T << 1;
        return T;
    } else if (n == 1) {
        Eigen::VectorXd T(2);
        T << 0, 1;
        return T;
    } else {
        Eigen::VectorXd Tn_1 = chebyshevToStandardBasis(n - 1);
        Eigen::VectorXd Tn_2 = chebyshevToStandardBasis(n - 2);

        Eigen::VectorXd Tn(Tn_1.size() + 1);
        Tn.setZero();

        // Compute 2t * T_{n-1}
        for (int i = 0; i < Tn_1.size(); ++i) {
            Tn[i + 1] += 2 * Tn_1[i];
        }

        // Subtract T_{n-2}
        for (int i = 0; i < Tn_2.size(); ++i) {
            Tn[i] -= Tn_2[i];
        }

        return Tn;
    }
}

// Main Conversion Function: Chebyshev coefficients to standard coefficients
Eigen::MatrixXd chebyshevToStandard(const Eigen::MatrixXd& cheb_coeffs) {
    int dim = cheb_coeffs.rows();
    int degree = cheb_coeffs.cols() - 1;

    // Build Chebyshev-to-Standard Transformation Matrix M
    Eigen::MatrixXd M(degree + 1, degree + 1);
    M.setZero();

    for (int i = 0; i <= degree; ++i) {
        Eigen::VectorXd Ti = chebyshevToStandardBasis(i);
        for (int j = 0; j < Ti.size(); ++j) {
            M(i, j) = Ti[j];
        }
    }

    // Transform coefficients
    Eigen::MatrixXd std_coeffs = cheb_coeffs * M;

    return std_coeffs;  // dim x (degree+1)
}

// Recursive function to compute Legendre polynomial P_n(t) in standard basis
Eigen::VectorXd legendreToStandardBasis(int n) {
    if (n == 0) {
        Eigen::VectorXd P(1);
        P << 1;
        return P;
    } else if (n == 1) {
        Eigen::VectorXd P(2);
        P << 0, 1;
        return P;
    } else {
        Eigen::VectorXd Pn_1 = legendreToStandardBasis(n - 1);
        Eigen::VectorXd Pn_2 = legendreToStandardBasis(n - 2);

        Eigen::VectorXd Pn(Pn_1.size() + 1);
        Pn.setZero();

        // Using recurrence: (n+1) P_{n+1}(t) = (2n+1) t P_n(t) - n P_{n-1}(t)
        // So P_n(t) = [(2n-1) t P_{n-1}(t) - (n-1) P_{n-2}(t)] / n

        // Compute (2n-1) * t * P_{n-1}(t)
        for (int i = 0; i < Pn_1.size(); ++i) {
            Pn[i + 1] += (2 * n - 1) * Pn_1[i];
        }

        // Subtract (n-1) * P_{n-2}(t)
        for (int i = 0; i < Pn_2.size(); ++i) {
            Pn[i] -= (n - 1) * Pn_2[i];
        }

        // Divide by n
        Pn /= n;

        return Pn;
    }
}

// Main Conversion Function: Legendre coefficients to standard coefficients
Eigen::MatrixXd legendreToStandard(const Eigen::MatrixXd& leg_coeffs) {
    int dim = leg_coeffs.rows();
    int degree = leg_coeffs.cols() - 1;

    // Build Legendre-to-Standard Transformation Matrix M
    Eigen::MatrixXd M(degree + 1, degree + 1);
    M.setZero();

    for (int i = 0; i <= degree; ++i) {
        Eigen::VectorXd Pi = legendreToStandardBasis(i);
        for (int j = 0; j < Pi.size(); ++j) {
            M(i, j) = Pi[j];
        }
    }

    // Transform coefficients
    Eigen::MatrixXd std_coeffs = leg_coeffs * M;

    return std_coeffs;  // dim x (degree+1)
}



// Recursive function to compute Laguerre polynomial L_n(t) in standard basis
Eigen::VectorXd laguerreToStandardBasis(int n) {
    if (n == 0) {
        Eigen::VectorXd L(1);
        L << 1;
        return L;
    } else if (n == 1) {
        Eigen::VectorXd L(2);
        L << 1, -1;  // L_1(t) = 1 - t
        return L;
    } else {
        Eigen::VectorXd L_n   = laguerreToStandardBasis(n - 1);
        Eigen::VectorXd L_n_1 = laguerreToStandardBasis(n - 2);

        Eigen::VectorXd L(L_n.size() + 1);
        L.setZero();

        // Compute (2n - 1) * L_{n-1}(t)
        for (int i = 0; i < L_n.size(); ++i) {
            L[i] += (2 * (n - 1) + 1) * L_n[i];
        }

        // Compute -t * L_{n-1}(t) (shift by 1 degree and multiply by -1)
        for (int i = 0; i < L_n.size(); ++i) {
            L[i + 1] -= L_n[i];
        }

        // Subtract (n-1) * L_{n-2}(t)
        for (int i = 0; i < L_n_1.size(); ++i) {
            L[i] -= (n - 1) * L_n_1[i];
        }

        // Divide by n
        L /= n;

        return L;
    }
}

// Main Conversion Function: Laguerre coefficients to standard coefficients
Eigen::MatrixXd laguerreToStandard(const Eigen::MatrixXd& lag_coeffs) {
    int dim = lag_coeffs.rows();
    int degree = lag_coeffs.cols() - 1;

    // Build Laguerre-to-Standard Transformation Matrix M
    Eigen::MatrixXd M(degree + 1, degree + 1);
    M.setZero();

    for (int i = 0; i <= degree; ++i) {
        Eigen::VectorXd Li = laguerreToStandardBasis(i);
        for (int j = 0; j < Li.size(); ++j) {
            M(i, j) = Li[j];
        }
    }

    // Transform coefficients
    Eigen::MatrixXd std_coeffs = lag_coeffs * M;

    return std_coeffs;  // dim x (degree+1)
}
