#include "cgaltools.h"

/** Solve the quadratic polynomial f.
 *
 * returns a pair of boolean values, <has_real_roots, is_square>.
 *   has_real_roots is true iff the polynomial has real (and not just complex) roots.
 *   is_square      is true iff the found (real) root has multiplicity 2.
 *
 * the function puts the result in arguments x0 and x1,
 * where after the function returns with has_real_roots set, x0 <= x1
 * (with x0 == x1 iff is_square).
 *
 * If has_real_roots is false, x0 and x1 remain unmodified
 *   and we always return false for is_square.
 */
std::pair<bool, bool>
solve_quadratic(const Polynomial_1& f, NT& x0, NT& x1) {
  std::pair<bool,bool> res;
  assert(f.degree() == 2);

  NT a(f[2]);
  NT b(f[1]);
  NT c(f[0]);
  //LOG(DEBUG) << " d: " << CGAL::to_double(a) << "t^2 + "
  //                    << CGAL::to_double(b) << "t + "
  //                    << CGAL::to_double(c);

  /* CGAL can factorize polynomials, but only square-free ones,
   * which doesn't help us much, alas.
   */
  NT radicand(b*b - 4. * a * c);
  if (radicand == 0) {
    x0 = x1 = (-b) / (2. * a);
    res = {true, true};
  } else if (radicand > 0) {
    NT root(CGAL::sqrt(radicand));
    NT divisor(2.*a);
    auto sign = f.sign();
    if (sign == CGAL::POSITIVE) {
      x0 = (-b - root) / divisor;
      x1 = (-b + root) / divisor;
    } else {
      assert(sign == CGAL::NEGATIVE);
      x1 = (-b - root) / divisor;
      x0 = (-b + root) / divisor;
    }
    //LOG(DEBUG) << " x0: " << CGAL::to_double(x0);
    //LOG(DEBUG) << " x1: " << CGAL::to_double(x1);
    //LOG(DEBUG) << " x0: " << x0;
    //LOG(DEBUG) << " x1: " << x1;
    res = {true, false};
  } else {
    res = {false, false};
  }

  return res;
}

#if 0
/* This code requires access to some internal CGAL data
 * structures, so it might require defining protected
 * as public and re-delcaring a few private things
 * as public in CGAL's source.
 */
static
void
debug_core_NTs_walk_tree(CORE::ExprRep const * a) {
  if (auto c = dynamic_cast<CORE::ConstRep const *>(a)) {
    std::cout << "E{" << a << "}";
    if (auto r = dynamic_cast<CORE::ConstRealRep const *>(a)) {
      const CORE::RealRep * rep = &(r->value.getRep());
      if (rep->ID() == CORE::REAL_BIGRAT) {
        CORE::BigRat rat(rep->BigRatValue());
        std::cout << "cr[" << rat.get_str() << "]";
      } else {
        std::cout << "c?[" <<  rep->ID() << "]";
      }
    } else {
      std::cout << "c[" << c->dump(CORE::ExprRep::VALUE_ONLY) << "]";
    }
  } else if (auto u = dynamic_cast<CORE::UnaryOpRep const *>(a)) {
    debug_core_NTs_walk_tree(u->child);
    std::cout << "E{" << a << "}";
    std::cout << "1" << u->op() <<
      "[" <<
      "E{" << u->child << "}" <<
      "]";
  } else if (auto b = dynamic_cast<CORE::BinOpRep const *>(a)) {
    debug_core_NTs_walk_tree(b->first);
    debug_core_NTs_walk_tree(b->second);
    std::cout << "E{" << a << "}";
    std::cout << "2" << b->op() <<
      "[" <<
      "E{" << b->first << "}" <<
      "," <<
      "E{" << b->second << "}" <<
      "]";
  }
  std::cout << "  # " << a->dump(CORE::ExprRep::VALUE_ONLY) << std::endl;
}
#endif
