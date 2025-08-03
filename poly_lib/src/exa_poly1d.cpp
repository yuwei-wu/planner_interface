#include <iostream>
#include "poly_lib/poly_1d.hpp"

int main() {
  poly_lib::Poly1D poly({1.0f, 0.5f, -0.2f}, poly_lib::Poly1D::STANDARD);

  for (double t = 0.0; t <= 2.0; t += 0.5) {
    std::cout << "t=" << t << ", val=" << poly.evaluate(t) << std::endl;
  }

  return 0;
}
