# planner_interface

`planner_interface` is a modular ROS package library providing polynomial and B-spline trajectory representations, along with ROS message definitions for trajectory planning. 

- **poly_lib**: ROS2 library for polynomial and B-spline computations.
- **traj_msgs**: Custom ROS message definitions to describe trajectories and splines.

---

## Repository Structure

```
├── poly_lib
│   ├── CMakeLists.txt
│   ├── include
│   │   └── poly_lib
│   │       ├── bspline.hpp
│   │       ├── poly_1d.hpp
│   │       ├── poly_nd.hpp
│   │       └── traj_data.hpp
│   ├── package.xml
│   └── src
│       ├── exa_poly1d.cpp
│       ├── exp_spline.cpp
│       └── poly_node.cpp
├── README.md
└── traj_msgs
    ├── CMakeLists.txt
    ├── msg
    │   ├── BdDervi.msg
    │   ├── Bspline.msg
    │   ├── Discrete.msg
    │   ├── MultiTraj.msg
    │   ├── PiecewisePoly.msg
    │   ├── Polynomial.msg
    │   └── SingleTraj.msg
    └── package.xml

````

---

## Features

### poly_lib
- Polynomial evaluation with multiple basis types: Standard, Chebyshev, Legendre, Hermite, Laguerre
- Multi-dimensional polynomial curves (`PolyND`)
- B-spline support (Bezier and Bernstein basis), Conversion from B-spline to polynomial form

### traj_msgs
- Custom message types describing various polynomial and spline trajectories
- Designed for uniform and non-uniform B-splines, multi-segment trajectories, and discrete waypoint trajectories

---

## Dependencies

- C++17 compatible compiler
- CMake 3.5+
- [Eigen3](https://eigen.tuxfamily.org/)
- ROS2

---

## Build Instructions

Build the entire workspace using `colcon` or `ament`:

```bash
colcon build 
````

---

## Resources 

### 1. Polynomial Bases 

| Polynomial Type     | Basis Functions                               | Domain            | Orthogonal? |
| ------------------- | --------------------------------------------- | ----------------- | ----------- |
| Standard (Monomial) | $t^k$                                         | Any real interval | No          |
| Chebyshev           | $T_k(t) = \cos(k \arccos t)$                  | $[-1,1]$          | Yes         |
| Bernstein (Bezier)  | $B_{k,n}(t) = \binom{n}{k} t^k (1 - t)^{n-k}$ | $[0,1]$           | No          |
| B-spline            | $N_{i,p}(t)$                                  | $[t_0, t_m]$      | No          |
| Legendre            | $P_k(t)$ (recurrence)                         | $[-1,1]$          | Yes         |
| Hermite             | $H_k(t)$ (Rodrigues’ formula)                 | $\mathbb{R}$      | Yes         |
| Laguerre            | $L_k(t)$ (Rodrigues’ formula)                 | $[0, \infty)$     | Yes         |

<details>
<summary><b>1. Standard (Monomial) Polynomials</b></summary>

**General Form:**  
$$p(t) = a_0 + a_1 t + a_2 t^2 + \dots + a_n t^n$$

- **Variables:** $t \in \mathbb{R}$, typically representing time or spatial variable.  
- **Domain:** Usually any real interval; no restriction.  
- **Description:**  
  The simplest and most intuitive polynomial basis using powers of $t$.  
- **Properties:**  
  - Basis functions: $\{1, t, t^2, \ldots, t^n\}$  
  - Not orthogonal, can suffer from numerical instability at high degrees (e.g., Runge's phenomenon).  
</details>

<details>
<summary><b>2. Chebyshev Polynomials</b></summary>

**General Form:**  
$$p(t) = \sum_{k=0}^n c_k T_k(t)$$

Where the Chebyshev polynomials $T_k(t)$ are defined by:  
$$T_0(t) = 1, \quad T_1(t) = t, \quad T_{k+1}(t) = 2 t T_k(t) - T_{k-1}(t)$$  
or equivalently:  
$$T_k(t) = \cos(k \arccos t)$$

- **Variables:** $t \in [-1, 1]$  
- **Domain:** Defined specifically on the interval $[-1,1]$. Inputs outside require domain scaling.  
- **Description:**  
  Chebyshev polynomials form an orthogonal basis with respect to the weight $\frac{1}{\sqrt{1 - t^2}}$.  
- **Properties:**  
  - Orthogonal basis → improved numerical stability.  
  - Minimizes the maximum error in polynomial approximation (minimax property).  
  - Helps reduce oscillations (Runge’s phenomenon).  
</details>

<details>
<summary><b>3. Bernstein Polynomials (Bezier Basis)</b></summary>

**General Form:**  
$$p(t) = \sum_{k=0}^n b_k B_{k,n}(t)$$

Where the Bernstein basis polynomials are:  
$$B_{k,n}(t) = \binom{n}{k} t^k (1 - t)^{n-k}$$

- **Variables:** $t \in [0, 1]$  
- **Domain:** Defined on $[0,1]$, which fits well with normalized curve parameterization.  
- **Description:**  
  The Bernstein basis is the foundation of Bezier curves. The coefficients $b_k$ are the control points.  
- **Properties:**  
  - Basis functions are positive and sum to 1 (good for shape control and convex hull property).  
  - Intuitive geometric interpretation with control points.  
</details>

<details>
<summary><b>4. B-spline </b></summary>

**General Form:**
$p(t) = \sum_{i=0}^m c_i N_{i,n}(t)$

Where the B-spline basis functions $N_{i,n}(t)$ are defined recursively over a knot vector $\{t_0, t_1, \ldots, t_{m+n+1}\}$:

- Zero-degree basis:
$N_{i,0}(t) = \text{1 if } t_i \leq t < t_{i+1}, \text{ else } 0$


- Recursive definition:
$N_{i,n}(t) = \frac{t - t_i}{t_{i+n} - t_i} N_{i,n-1}(t) + \frac{t_{i+n+1} - t}{t_{i+n+1} - t_{i+1}} N_{i+1,n-1}(t)$

* **Variables:** $t \in [t_n, t_{m+1}]$ (domain determined by knot vector)
* **Description:**
  B-splines generalize Bezier curves by piecing together polynomial segments with continuity and local control, where each basis function is nonzero only on a limited interval defined by knots.
* **Properties:**
  * Local support: each basis function affects only a portion of the curve, allowing local shape modification.
  * Partition of unity: basis functions sum to 1 everywhere on the domain.
  * Flexibility: knot multiplicities control continuity and shape.

</details>

<details>
<summary><b>5. Legendre Polynomials</b></summary>

**General Form:**  
$$p(t) = \sum_{k=0}^n l_k P_k(t)$$

Where Legendre polynomials $P_k(t)$ satisfy the recurrence:  
$$(k+1) P_{k+1}(t) = (2k+1) t P_k(t) - k P_{k-1}(t)$$

- **Variables:** $t \in [-1, 1]$  
- **Domain:** Orthogonal on $[-1,1]$ with uniform weight.  
- **Description:**  
  Another set of orthogonal polynomials commonly used in physics and numerical integration (Gauss-Legendre quadrature).  
- **Properties:**  
  - Orthogonal w.r.t. uniform weight → useful for approximation and solving differential equations.  
</details>

<details>
<summary><b>6. Hermite Polynomials</b></summary>

**General Form:**  
$$p(t) = \sum_{k=0}^n h_k H_k(t)$$

Where Hermite polynomials $H_k(t)$ can be defined via Rodrigues’ formula:  
$$H_k(t) = (-1)^k e^{t^2} \frac{d^k}{dt^k} e^{-t^2}$$

- **Variables:** $t \in \mathbb{R}$ (all real numbers)  
- **Domain:** Entire real line.  
- **Description:**  
  Orthogonal polynomials with respect to Gaussian weight $e^{-t^2}$.  
- **Properties:**  
  - Useful for approximations involving Gaussian weight functions.  
</details>

<details>
<summary><b>6. Laguerre Polynomials</b></summary>

**General Form:**  
$$p(t) = \sum_{k=0}^n g_k L_k(t)$$

Where Laguerre polynomials satisfy:  
$$L_k(t) = \frac{e^t}{k!} \frac{d^k}{dt^k} \left(t^k e^{-t}\right)$$

- **Variables:** $t \in [0, \infty)$  
- **Domain:** Semi-infinite interval $[0, \infty)$.  
- **Description:**  
  Orthogonal polynomials with respect to exponential decay weight $e^{-t}$.  
- **Properties:**  
  - Suitable for problems on infinite or semi-infinite domains.  
</details>

---

## Acknowledges

- Thanks to the [OpenAI GPT](https://openai.com/) for assistance in developing the code and documentation.
- Reference: 
   -  https://github.com/ethz-asl/mav_comm
   -  https://github.com/KumarRobotics/kr_mav_control/tree/poly/trackers/kr_tracker_msgs
   -  https://github.com/KumarRobotics/kr_mav_control/tree/master/kr_mav_msgs
   -  https://github.com/KumarRobotics/kr_planning_msgs
   -  https://github.com/ZJU-FAST-Lab/EGO-Planner-v2/tree/main/swarm-playground/main_ws/src/planner/traj_utils


