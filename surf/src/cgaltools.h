/**  surfer2 -- a straight skeleton implementation
 *
 *  Copyright 2018, 2019 Peter Palfraader
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#pragma once

#include "surf.h"

/* cgal things */
#ifndef NT_USE_DOUBLE
  #include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
  using Kernel  = CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt;
  using NT = Kernel::FT;

  static const NT CORE_ONE =  NT::getOne();
  static const NT CORE_ZERO = NT::getZero();

  #define string_to_maybe_NT(x) x
#else
  #if 0
    #include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
    using Kernel  = CGAL::Exact_predicates_inexact_constructions_kernel;
    using NT = Kernel::FT;

    static const NT CORE_ONE =  1.0;
    static const NT CORE_ZERO = 0.0;
    #define string_to_maybe_NT(x) std::stod(x)
  #else
    #include <CGAL/Cartesian.h>
    using Kernel  = CGAL::Cartesian<double>;
    using NT = Kernel::FT;

    static const NT CORE_ONE =  1.0;
    static const NT CORE_ZERO = 0.0;

    #define string_to_maybe_NT(x) std::stod(x)
  #endif
#endif


#ifdef NT_USE_DOUBLE
  #define assert_sign(X) STMT_NIL
  inline bool compare_NT_real_eq(const NT& a, const NT& b) {
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wfloat-equal"
    return (a==b);
    #pragma GCC diagnostic pop
  }
#else
  inline bool compare_NT_real_eq(const NT& a, const NT& b) {
    return (a==b);
  }
  #define assert_sign(X) { \
    const NT x__(X); \
    assert(x__.Rep()->getSign() == x__.Rep()->getExactSign()); \
  }
#endif

#define assert_ge(X, Y) { \
  const NT delta__((X) - (Y)); \
  assert_sign(delta__); \
  assert(delta__ >= 0); \
}

#include <CGAL/Algebraic_kernel_d_1.h>
#include <CGAL/Plane_3.h>
#include <utility>

using Point_2 = typename Kernel::Point_2;
using Line_2 = typename Kernel::Line_2;
using Segment_2 = typename Kernel::Segment_2;
using Vector_2 = typename Kernel::Vector_2;

using Point_3 = typename Kernel::Point_3;
using Segment_3 = typename Kernel::Segment_3;
using Ray_3 = typename Kernel::Ray_3;
using Vector_3 = typename Kernel::Vector_3;
using Plane_3 = typename Kernel::Plane_3;


inline
std::string
CGAL_point(const Point_2& p) {
  std::ostringstream oss;
  oss << "(" << CGAL::to_double(p.x()) <<
         " " << CGAL::to_double(p.y()) <<
         ")";
  return oss.str();
}

inline
std::string
CGAL_point(const Point_3& p) {
  std::ostringstream oss;
  oss << "(" << CGAL::to_double(p.x()) <<
         " " << CGAL::to_double(p.y()) <<
         " " << CGAL::to_double(p.z()) <<
         ")";
  return oss.str();
}

inline
std::string
CGAL_vector(const Vector_2& p) {
  std::ostringstream oss;
  oss << "(" << CGAL::to_double(p.x()) <<
         " " << CGAL::to_double(p.y()) <<
         ")";
  return oss.str();
}

inline
std::string
CGAL_line(const Line_2& l) {
  std::ostringstream oss;
  oss << "(" << CGAL_point(l.point(0)) <<
         "-" << CGAL_point(l.point(1)) <<
         "; " << CGAL::to_double(l.a()) << "x + "
             << CGAL::to_double(l.b()) << "y + "
             << CGAL::to_double(l.c()) << " == 0" <<
         ")";
  return oss.str();
}

#ifndef NT_USE_DOUBLE
using AlgebraicKernel     = CGAL::Algebraic_kernel_d_1<NT>;
using Polynomial_1        = AlgebraicKernel::Polynomial_1;
using Algebraic_real_1    = AlgebraicKernel::Algebraic_real_1;
using Polynomial_traits_1 = CGAL::Polynomial_traits_d< Polynomial_1 >;

using CGAL::differentiate;
using CGAL::evaluate;
#else /* ifdef NT_USE_DOUBLE */
class Polynomial_1 {
  const NT a_[3];
  unsigned degree_;
public:
  Polynomial_1(const NT& a0=CORE_ZERO, const NT& a1=CORE_ZERO, const NT& a2=CORE_ZERO)
    : a_{a0, a1, a2}
    , degree_(!compare_NT_real_eq(a2, CORE_ZERO) ? 2 :
              !compare_NT_real_eq(a1, CORE_ZERO) ? 1 :
                                                   0)
  { }
  unsigned degree() const { return degree_; };
  CGAL::Sign sign() const { return CGAL::sign(a_[degree_]); };
  const NT& operator[] (unsigned index) const {
    assert(index <= degree_);
    return a_[index];
  }
  Polynomial_1 differentiate() const {
    return Polynomial_1(a_[1], 2*a_[2]);
  }
  NT evaluate(const NT& at) const {
    return a_[2] * (at*at)
         + a_[1] * at
         + a_[0];
  }
  Polynomial_1 operator*(const Polynomial_1& o) const {
    assert(degree_ + o.degree_ <= 2);
    return Polynomial_1(a_[0]*o.a_[0],
                        a_[0]*o.a_[1] + a_[1]*o.a_[0],
                        a_[0]*o.a_[2] + a_[1]*o.a_[1] + a_[2]*o.a_[0]);
  }
  Polynomial_1 operator+(const Polynomial_1& o) const {
    return Polynomial_1(a_[0]+o.a_[0],
                        a_[1]+o.a_[1],
                        a_[2]+o.a_[2]);
  }
  Polynomial_1 operator-(const Polynomial_1& o) const {
    return Polynomial_1(a_[0]-o.a_[0],
                        a_[1]-o.a_[1],
                        a_[2]-o.a_[2]);
  }
  friend std::ostream& operator<<(std::ostream&, const Polynomial_1&);
};
inline Polynomial_1
differentiate(const Polynomial_1& p) {
  return p.differentiate();
}
inline std::ostream& operator<<(std::ostream& o, const Polynomial_1& p) {
  switch (p.degree_) {
    case 0:
      return o << "(" << p.a_[0] << ")";
      break;
    case 1:
      return o << "(" << p.a_[1] << "*x + " << p.a_[0] << ")";
      break;
    default:
      return o << "(" << p.a_[2] << "*x^2 + " << p.a_[1] << "*x + " << p.a_[0] << ")";
      break;
  };
}
inline NT
evaluate(const Polynomial_1& p, const NT& at) {
  return p.evaluate(at);
};
#endif /* ifdef NT_USE_DOUBLE */


std::pair<bool,bool> solve_quadratic(const Polynomial_1& f, NT& x0, NT& x1);
