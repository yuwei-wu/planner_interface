#include <chrono>
#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <poly_lib/poly_nd.hpp>  // Use PolyND header

using namespace std::chrono_literals;

class PolyNode : public rclcpp::Node {
public:
  PolyNode() : Node("poly_node") {
    RCLCPP_INFO(this->get_logger(), "poly_node started");

    int dimensions = 2;          // 2D polynomial
    int order = 3;               // Cubic polynomial
    int segments = 2;            // Two segments
    poly_lib::Poly1D::BasisType basis = poly_lib::Poly1D::BasisType::STANDARD;


    poly_ = std::make_shared<poly_lib::PolyND>(dimensions, order, segments, basis);

    poly_->setSegDurations({1.0, 2.0});

    std::vector<std::vector<float>> coefs_seg0 = {
      {1.0f, 0.0f, 0.0f, 0.0f},   // x(t) = 1 (constant)
      {0.0f, 1.0f, 0.0f, 0.0f}    // y(t) = t (linear)
    };
    std::vector<std::vector<float>> coefs_seg1 = {
      {0.0f, 0.0f, 1.0f, 0.0f},   // x(t) = t^2 (quadratic)
      {1.0f, 0.0f, 0.0f, 0.0f}    // y(t) = 1 (constant)
    };

    poly_->setSegCoefficients(0, coefs_seg0);
    poly_->setSegCoefficients(1, coefs_seg1);

    timer_ = this->create_wall_timer(
      500ms, [this]() {
        static double t = 0.0;
        auto val = poly_->evaluate(t);
        RCLCPP_INFO(this->get_logger(), "t=%.2f: poly value = [%.3f, %.3f]", t, val(0), val(1));
        t += 0.1;
        if (t > 3.0) {
          t = 0.0;
        }
      }
    );
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<poly_lib::PolyND> poly_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PolyNode>());
  rclcpp::shutdown();
  return 0;
}
