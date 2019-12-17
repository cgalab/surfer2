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
#include "KineticTriangle.h"
#include "WavefrontVertex.h"

unsigned KineticTriangle::ktctr = 0;

#ifndef NDEBUG
void
KineticTriangle::
assert_valid() const { //{{{
  assert(!is_dead_);
  DBG_FUNC_BEGIN(DBG_TRIANGLE_ASSERT_VALID);
  DBG(DBG_TRIANGLE_ASSERT_VALID) << this;
  for (int i=0; i<3; ++i) {
    DBG(DBG_TRIANGLE_ASSERT_VALID) << "- v" << i << ": " << vertices[i]->details();
  }
  for (int i=0; i<3; ++i) {
    assert(vertices[i]); // "Missing vertex %d", i
    assert(!!neighbors[i] == !wavefronts[i]); // "Wavefront vs. neighbor existence mismatch at %d", i

    if (neighbors[i]) {
      /* Not a constraint. */
      auto const * const n = neighbors[i];
      DBG(DBG_TRIANGLE_ASSERT_VALID) << "- " << i << ": - checking neighbor " << n;
      assert(n->has_neighbor(this)); // "Neighborhood relation inconsistent"
      int n_idx = n->index(this);
      DBG(DBG_TRIANGLE_ASSERT_VALID) << "     - checking edge/vertex match: 1  " << vertices[cw (i)] << " vs " << n->vertices[ccw(n_idx)];
      DBG(DBG_TRIANGLE_ASSERT_VALID) << "     - checking edge/vertex match: 2  " << vertices[ccw(i)] << " vs " << n->vertices[cw (n_idx)];
      assert(vertices[cw (i)] == n->vertices[ccw(n_idx)]); // "Edge vertex mismatch"
      assert(vertices[ccw(i)] == n->vertices[cw (n_idx)]); // "Edge vertex mismatch"
    } else {
      DBG(DBG_TRIANGLE_ASSERT_VALID) << "- " << i << ": checking wavefront";
      assert(wavefronts[i]);
      DBG(DBG_TRIANGLE_ASSERT_VALID) << "- " << i << ":                   : " << *wavefronts[i];
      assert(wavefronts[i]->incident_triangle());
      assert(wavefronts[i]->incident_triangle() == this);
      assert(! wavefronts[i]->is_dead());

      /* a wavefront has vertices */
      assert(wavefronts[i]->vertex(0) == vertices[ccw(i)]);
      assert(wavefronts[i]->vertex(1) == vertices[cw (i)]);

      /* a vertex has wavefronts */
      assert(vertices[ccw(i)]->wavefronts()[1] == wavefronts[i]);
      assert(vertices[cw (i)]->wavefronts()[0] == wavefronts[i]);

      // XXX check if input edge oriented correctly (same as constraint)
    }
  }
  DBG_FUNC_END(DBG_TRIANGLE_ASSERT_VALID);
}//}}}
#endif

CollapseSpec
KineticTriangle::
compute_collapse(const NT& time_now) const { // {{{
  DBG_FUNC_BEGIN(DBG_TRIANGLE);
  DBG(DBG_TRIANGLE) << this;
  CollapseSpec result(component);
  InfiniteSpeedType infinite_speed_type = has_vertex_infinite_speed();
  if (infinite_speed_type == InfiniteSpeedType::NONE) {
    if (unbounded()) {
      result = compute_collapse_unbounded(time_now);
    } else {
      result = compute_collapse_bounded(time_now);
    }
  } else if (infinite_speed_type == InfiniteSpeedType::OPPOSING) {
    result = CollapseSpec(component, CollapseType::FACE_HAS_INFINITELY_FAST_VERTEX_OPPOSING, time_now);
  } else {
    assert(infinite_speed_type == InfiniteSpeedType::WEIGHTED);
    /* Only triangles with wavefront edges incident to the infinitely fast weighted vertex
     * need to witness this.  As a secondary key, we prefer the triangle with the faster edge
     * since that edge wins.
     */
    int relevant_edge = -1;
    NT relevant_edge_speed;
    for (int i=0; i<3; ++i) {
      if (!wavefronts[i]) continue;
      assert(wavefronts[i]->vertex(0) == vertices[ccw(i)]);
      assert(wavefronts[i]->vertex(1) == vertices[cw (i)]);
      if ((vertices[ccw(i)]->infinite_speed != InfiniteSpeedType::WEIGHTED) &&
          (vertices[cw (i)]->infinite_speed != InfiniteSpeedType::WEIGHTED)) continue;

      if (relevant_edge < 0 ||
        relevant_edge_speed < wavefronts[i]->l()->weight * FASTER_EDGE_WINS_IN_COLLINEAR_CASES) {
        relevant_edge = i;
        relevant_edge_speed = wavefronts[i]->l()->weight * FASTER_EDGE_WINS_IN_COLLINEAR_CASES;
      }
    }
    if (relevant_edge < 0) {
      result = CollapseSpec(component, CollapseType::INVALID_EVENT, time_now);
    } else {
      result = CollapseSpec(component, CollapseType::FACE_HAS_INFINITELY_FAST_VERTEX_WEIGHTED, time_now, relevant_edge, relevant_edge_speed);
    }
  }
  DBG(DBG_TRIANGLE) << this << " returning " << result;
  DBG_FUNC_END(DBG_TRIANGLE);
  return result;
} // }}}

/** Checks what kind, if any, of infinitely fast vertices this triangle has.
 *
 * Returns OPPOSING if there is at least one vertex of that type, else,
 * WEIGHTED  if there is at least one vertex of that, else NONE.
 */
InfiniteSpeedType
KineticTriangle::
has_vertex_infinite_speed() const { // {{{{
  if (vertex(0)->infinite_speed == InfiniteSpeedType::NONE &&
      vertex(1)->infinite_speed == InfiniteSpeedType::NONE &&
      vertex(2)->infinite_speed == InfiniteSpeedType::NONE) {
    return InfiniteSpeedType::NONE;
  } else if (vertex(0)->infinite_speed == InfiniteSpeedType::OPPOSING ||
      vertex(1)->infinite_speed == InfiniteSpeedType::OPPOSING ||
      vertex(2)->infinite_speed == InfiniteSpeedType::OPPOSING) {
    return InfiniteSpeedType::OPPOSING;
  } else {
    assert(vertex(0)->infinite_speed == InfiniteSpeedType::WEIGHTED ||
           vertex(1)->infinite_speed == InfiniteSpeedType::WEIGHTED ||
           vertex(2)->infinite_speed == InfiniteSpeedType::WEIGHTED);
    return InfiniteSpeedType::WEIGHTED;
  }
} // }}}

CollapseSpec
KineticTriangle::
event_that_will_not_happen(int component, const NT& time_now, const Polynomial_1& determinant) { // {{{
  DBG_FUNC_BEGIN(DBG_TRIANGLE);
  CollapseSpec result(component);

  #ifdef DEBUG_COLLAPSE_TIMES
    NT collapse_time;
    bool has_collapse = get_generic_collapse_time(time_now, determinant, collapse_time);
    if (has_collapse) {
      result = CollapseSpec(component, CollapseType::INVALID_EVENT, collapse_time);
      DBG(DBG_TRIANGLE_TIMING) << "Putting something into the priority queue anyway at determinant's correct zero.";
    } else {
      result = CollapseSpec(component, CollapseType::NEVER);
    }
  #else
    result = CollapseSpec(component, CollapseType::NEVER);
  #endif

  DBG(DBG_TRIANGLE) << "returning " << result;
  DBG_FUNC_END(DBG_TRIANGLE);
  return result;
} // }}}

/** Check if a vertex is moving faster or slower relative to an edge.
 *
 * If the vertex is in front of the edge, and the edge is faster
 * (CGAL::POSITIVE), the edge may eventually catch up, causing a split
 * or flip event.
 *
 * If the vertex is faster then the edge (i.e. if the edge is slower,
 * CGAL::NEGATIVE), and if the vertex is behind, v might overtake the edge
 * causing a flip event.
 *
 * If the edge is slower, return -1.  If it's faster +1.  If the same speed, 0.
 *
 * (As a result, if they move in opposite directions, then e is "faster" and +1 is returned.)
 */
CGAL::Sign
KineticTriangle::
edge_is_faster_than_vertex(const WavefrontVertex& v, const WavefrontSupportingLine& e) { // {{{
  DBG_FUNC_BEGIN(DBG_TRIANGLE_TIMING2);

  /* Let n be some normal to e,
   * let s be the speed (well, velocity) vector of the vertex v.
   * let w be the weight (speed) of e.
   */
  const Vector_2& n(e.normal_direction);
  const Vector_2& s(v.velocity);
  const NT& w(e.weight);
  /* Then s.n is the length of the projection of s onto n, times the length of n.
   * Per time unit, v and e will approach each other by (w - s.n/|n|).
   */

  const NT scaled_edge_speed   = w * CGAL::sqrt(n.squared_length());
  const NT scaled_vertex_speed = s * n;

  const NT speed_approach = scaled_edge_speed - scaled_vertex_speed;
  const CGAL::Sign sign = CGAL::sign(speed_approach);

  DBG(DBG_TRIANGLE_TIMING2) << "returning " << sign;
  DBG_FUNC_END(DBG_TRIANGLE_TIMING2);
  return sign;
} // }}}

/** Compute when the vertex v will move over the supporting line of e (or crash into e).
 *
 * We only call this function when we have a triangle with exactly one constraint.
 *
 * The resulting time might also be now or be in the past.
 */
std::tuple<NT, KineticTriangle::VertexOnSupportingLineType>
KineticTriangle::
get_time_vertex_on_supporting_line(const WavefrontVertex& v, const WavefrontSupportingLine& e) { // {{{
  DBG_FUNC_BEGIN(DBG_TRIANGLE_TIMING2);
  NT collapse_time;
  VertexOnSupportingLineType vertex_on_line_type;

  /* Let n be some normal to e,
   * let P be some point on e (at time zero),
   * let s be the speed (well, velocity) vector of the vertex v.
   * let Q be the location of v at time zero, and
   * let w be the weight (speed) of e.
   */
  const Vector_2& n(e.normal_direction);
  const Point_2& P(e.l.point());
  const Vector_2& s(v.velocity);
  const Point_2& Q(v.pos_zero);
  const NT& w(e.weight);
  /* Then PQ.n is the length of the projection of PQ onto n, times the length of n.
   * Likewise, s.n is the length of the projection of s onto n, times the length of n.
   * Per time unit, v and e will approach each other by (w - s.n/|n|).
   *
   * So v will hit (the supporting line of) e at time t := PQ.n/|n| / (w - s.n/|n|) ==
   * == PQ.n / (w |n| - s.n)
   */
  const Vector_2 PQ(P, Q);
  const NT scaled_distance = PQ * n;
  const NT scaled_edge_speed   = w * CGAL::sqrt(n.squared_length());
  const NT scaled_vertex_speed = s * n;
  const NT scaled_speed_approach = scaled_edge_speed - scaled_vertex_speed;
  /*
  */
  DBG(DBG_TRIANGLE_TIMING2) << " -- n: " << CGAL_vector(n);
  DBG(DBG_TRIANGLE_TIMING2) << " -- P: " << CGAL_point(P);
  DBG(DBG_TRIANGLE_TIMING2) << " -- s: " << CGAL_vector(s);
  DBG(DBG_TRIANGLE_TIMING2) << " -- Q: " << CGAL_point(Q);
  DBG(DBG_TRIANGLE_TIMING2) << " -- w: " << CGAL::to_double(w);
  DBG(DBG_TRIANGLE_TIMING2) << " -- PQ: " << CGAL_vector(PQ);
  DBG(DBG_TRIANGLE_TIMING2) << " -- num (∝ distance      ): " << CGAL::to_double(scaled_distance);
  DBG(DBG_TRIANGLE_TIMING2) << " -- den (∝ approach speed): " << CGAL::to_double(scaled_speed_approach);

  if (scaled_speed_approach == 0) {
    collapse_time = CGAL::ZERO;
    if (scaled_distance == 0) {
      vertex_on_line_type = VertexOnSupportingLineType::ALWAYS;
    } else {
      vertex_on_line_type = VertexOnSupportingLineType::NEVER;
    }
  } else {
    collapse_time = scaled_distance/scaled_speed_approach;
    vertex_on_line_type = VertexOnSupportingLineType::ONCE;
  }

  DBG(DBG_TRIANGLE_TIMING2) << "returning " << CGAL::to_double(collapse_time) << " with VertexOnSupportingLineType " << vertex_on_line_type;
  DBG_FUNC_END(DBG_TRIANGLE_TIMING2);
  return std::make_tuple(collapse_time, vertex_on_line_type);
} // }}}

#if 0
/** determine if split or flip event
 *
 * The triangle has one constraint, e, and opposite vertex v
 * move onto the supporting line of e at time collapse_time.
 *
 * Determine if this is a split or a flip event.
 */
CollapseSpec
KineticTriangle::
determine_split_or_flip_bounded_constrained_1(const NT& collapse_time, unsigned c_idx) const {
  CollapseSpec result(component);
  result = CollapseSpec(component, CollapseType::SPLIT_OR_FLIP_REFINE, collapse_time, c_idx);
  return result;
}
#endif

/** find potential split or flip_event.
 *
 * We have a triangle with exactly one constraint, e, and opposite vertex v.
 * We don't like the collapse of e.  This can happen if e has
 * two parallel endpoints (so it does not collapse) or it witnesses
 * the wrong root/zero of the determinant polynomial of degree two.
 *
 * So check
 *  - if v crashes into e, or
 *  - if a vertex incident to e moves over a spoke (as v moves over the
 *    supporting line of e).
 */
CollapseSpec
KineticTriangle::
compute_split_or_flip_event_bounded_constrained_1(const NT& time_now, unsigned c_idx, const Polynomial_1& determinant) const { // {{{
  DBG_FUNC_BEGIN(DBG_TRIANGLE);
  DBG(DBG_TRIANGLE) << this;
  CollapseSpec result(component);

  assert(c_idx < 3);
  assert(wavefronts[c_idx] != NULL);
  assert(wavefronts[cw(c_idx)] == NULL);
  assert(wavefronts[ccw(c_idx)] == NULL);

  /* If all of the vertices are convex, this can't happen. */
  if (!vertices[0]->is_reflex_or_straight() &&
      !vertices[1]->is_reflex_or_straight() &&
      !vertices[2]->is_reflex_or_straight()) {
    DBG(DBG_TRIANGLE_TIMING) << this << " all convex vertices.  Will never see an event.";
    result = event_that_will_not_happen(component, time_now, determinant);
  } else  {
    const WavefrontSupportingLine &e(*wavefront(c_idx)->l());
    const WavefrontVertex v(*vertex(c_idx));
    NT collapse_time;
    VertexOnSupportingLineType vertex_on_line_type;
    std::tie(collapse_time, vertex_on_line_type) = get_time_vertex_on_supporting_line(v, e);
    switch (vertex_on_line_type) {
      case VertexOnSupportingLineType::ONCE:
        if (collapse_time > time_now) {
          DBG(DBG_TRIANGLE_TIMING) << " v will hit supporting line of e at time " << CGAL::to_double(collapse_time);
          result = CollapseSpec(component, CollapseType::SPLIT_OR_FLIP_REFINE, collapse_time, c_idx);
        } else if (collapse_time == time_now) {
          DBG(DBG_TRIANGLE_TIMING) << " v is on the supporting line of e right now " << CGAL::to_double(collapse_time);
          if (determinant.degree() == 2) {
            DBG(DBG_TRIANGLE_TIMING) << " determinant degree 2";
            if (accept_collapse_bounded_constrained_1(collapse_time, determinant, false)) {
              DBG(DBG_TRIANGLE_TIMING) << " Will want to handle this.";
              result = CollapseSpec(component, CollapseType::SPLIT_OR_FLIP_REFINE, collapse_time, c_idx);
            } else {
              DBG(DBG_TRIANGLE_TIMING) << " But the triangle is growing.";
              result = CollapseSpec(component, CollapseType::NEVER);
            }
          } else {
            assert(determinant.degree() == 1);
            CGAL::Sign sign(determinant.sign());

            DBG(DBG_TRIANGLE_TIMING) << " determinant degree 1, sign " << sign;
            if (sign == CGAL::NEGATIVE) {
              DBG(DBG_TRIANGLE_TIMING) << " Will want to handle this.";
              result = CollapseSpec(component, CollapseType::SPLIT_OR_FLIP_REFINE, collapse_time, c_idx);
            } else {
              LOG(WARNING) << "Untested code path.";
              DBG(DBG_TRIANGLE_TIMING) << " But the triangle is growing.";
              result = CollapseSpec(component, CollapseType::NEVER);
            }
          }
        } else {
          DBG(DBG_TRIANGLE_TIMING) << " v will not hit supporting line of e.";
          result = event_that_will_not_happen(component, time_now, determinant);
     //     assert(result.type() == CollapseType::NEVER); // XXX if this holds, we can drop the event_that_will_not_happen thing
        }
        break;
      case VertexOnSupportingLineType::NEVER:
        DBG(DBG_TRIANGLE_TIMING) << " v will never hit supporting line of e as they have the same speed";
        result = CollapseSpec(component, CollapseType::NEVER);
        break;
      case VertexOnSupportingLineType::ALWAYS:
        DBG(DBG_TRIANGLE_TIMING) << " v is on the supporting line of e and just as fast.  Event now.";
        result = CollapseSpec(component, CollapseType::SPLIT_OR_FLIP_REFINE, time_now, c_idx);
        break;
    }
  }
  DBG(DBG_TRIANGLE) << this << " returning " << result;
  DBG_FUNC_END(DBG_TRIANGLE);
  return result;
} // }}}

/** Learn when the triangle will collapse from just the determinant, with no
 * respect to combinatorics or other geometry.
 *
 * The determinant polynomial passed should be (positively proportional to) the
 * triangle's (signed) area.
 *
 * It must be of at most degree 2 (which is the case in our use).
 *
 *
 * We return whether this triangle will ever collapse (now or in the future),
 * along with the collapse time in the collapse_time argument.  Collapses in the past
 * are disregarded.
 *
 * Additionally, we note some extra information that helps the calling function classify the
 * type of collapse.
 */
bool
KineticTriangle::
get_generic_collapse_time(const NT& time_now, const Polynomial_1& det, NT& collapse_time) { // {{{
  DBG_FUNC_BEGIN(DBG_TRIANGLE_TIMING2);

  bool result;
#ifdef NT_USE_DOUBLE
  static NT last_time = 0;
  static unsigned count = 0;
#endif

  CGAL::Sign sign(det.sign());
  DBG(DBG_TRIANGLE_TIMING2) << "polynomial has degree " << det.degree() << " and sign " << sign;
  if (det.degree() == 0) {
    /*
     * This should really only happen when collinear points on the convex hull move such that they stay collinear.
     *
     * No need to switch things around, then.
     * If they change order, we should, eventually, catch that elsewhere because the vertices become incident.
     */
    LOG(WARNING) << "Have a polynomial of degree zero, Can we catch this sooner?  at time " << CGAL::to_double(time_now);
    if (CGAL::sign(det[0]) == CGAL::ZERO) {
      result = true;
      LOG(WARNING) << " collapses now (and always) too.";
#ifdef NT_USE_DOUBLE
if (time_now == last_time) {
  ++count;
  if (count > 1000) {
    LOG(ERROR) << "In double loop at line " << __FILE__ << ":" << __LINE__;
    abort();
  };
} else {
  count = 0;
  last_time = time_now;
};
#endif
    } else {
      result = false;
    }
  } else if (det.degree() == 1) {
    assert(sign != CGAL::ZERO);
    collapse_time = - det[0]/det[1];
    if (collapse_time == time_now) {
      if (sign == CGAL::POSITIVE) {
        DBG(DBG_TRIANGLE_TIMING2) << "Triangle area is zero and increasing, not using this collapse time";
        result = false;
      } else {
        LOG(WARNING) << "Polynomial (of degree 1) has a zero right right now.  Can we catch this sooner?.  at time " << CGAL::to_double(time_now);
        DBG(DBG_TRIANGLE_TIMING2) << "Triangle area is zero and decreasing, using this collapse time";
#ifdef NT_USE_DOUBLE
if (time_now == last_time) {
  ++count;
  if (count > 1000) {
    LOG(ERROR) << "In double loop at line " << __FILE__ << ":" << __LINE__;
    abort();
  };
} else {
  count = 0;
  last_time = time_now;
};
#endif
        result = true;
      }
    } else if (collapse_time > time_now) {
      assert(sign == CGAL::NEGATIVE);
      result = true;
      DBG(DBG_TRIANGLE_TIMING2) << "Triangle area is polynomial of degree one, and zero is in the future.  Using this.";
    } else {
      DBG(DBG_TRIANGLE_TIMING2) << "Triangle area is polynomial of degree one, and zero is in the past.  Not using this.";
      assert(sign == CGAL::POSITIVE);
      result = false;
    };
  } else {
    assert(det.degree() == 2);
    assert(sign != CGAL::ZERO);
    DBG(DBG_TRIANGLE_TIMING2)
                         << CGAL::to_double(det[2]) << ".t^2 + "
                         << CGAL::to_double(det[1]) << ".t + "
                         << CGAL::to_double(det[0]);


    NT x0, x1;
    DBG(DBG_TRIANGLE_TIMING2) << "solving quadratic.";
    bool has_real_roots, is_square;
    std::tie(has_real_roots, is_square) = solve_quadratic(det, x0, x1);
    if (! has_real_roots) {
      DBG(DBG_TRIANGLE_TIMING2) << "no real solutions.  sign is " << sign;
      assert(sign == CGAL::POSITIVE);
      result = false;
    } else {
      DBG(DBG_TRIANGLE_TIMING2) << "have real solutions (" << CGAL::to_double(x0) << ", " << CGAL::to_double(x1) << ").  Checking if we like them.";

      /*
      DBG(DBG_TRIANGLE_TIMING2) << " - x0:  " << CGAL::to_double(x0);
      DBG(DBG_TRIANGLE_TIMING2) << " - x1:  " << CGAL::to_double(x1);
      DBG(DBG_TRIANGLE_TIMING2) << " - now: " << CGAL::to_double(time_now);
      */

      assert (sign == CGAL::NEGATIVE || sign == CGAL::POSITIVE);
      if (sign == CGAL::NEGATIVE) {
        DBG(DBG_TRIANGLE_TIMING2) << " we like x1: The sign of the determinant is negative.  So the second root must be a valid event.";
        collapse_time = x1;
        assert(x1 >= time_now);
        result = true;
      } else if (x0 >= time_now) {
        DBG(DBG_TRIANGLE_TIMING2) << " we like x0: The sign of the determinant is positive and the first root is not in the past.";
        collapse_time = x0;
        result = true;
      } else {
        DBG(DBG_TRIANGLE_TIMING2) << " we like neither: The sign of the determinant is positive, but the first root is in the past.";
        result = false;
      }
    }
  }
  if (result) {
    DBG(DBG_TRIANGLE) << "returning " << result << " with " << CGAL::to_double(collapse_time);
  } else {
    DBG(DBG_TRIANGLE) << "returning " << result;
  }
  DBG_FUNC_END(DBG_TRIANGLE_TIMING2);
  return result;
} // }}}

Polynomial_1
KineticTriangle::
compute_determinant_from_vertices(const WavefrontVertex * const v0, const WavefrontVertex * const v1, const WavefrontVertex * const v2) { // {{{
  assert(v0);
  assert(v1);
  assert(v2);
  return compute_determinant(
          v0->px(), v0->py(),
          v1->px(), v1->py(),
          v2->px(), v2->py());
} // }}}

/** only called for unconstrained triangles, only from compute_flip_event
 *
 * Can return NEVER, TRIANGLE_COLLAPSE, SPOKE_COLLAPSE, or VERTEX_MOVES_OVER_SPOKE.
 */
CollapseSpec
KineticTriangle::
get_generic_collapse(const NT& time_now, const Polynomial_1& determinant) const { // {{{
  DBG_FUNC_BEGIN(DBG_TRIANGLE);
  DBG(DBG_TRIANGLE) << this;

  assert(!is_constrained(0) && !is_constrained(1) && !is_constrained(2));

  CollapseSpec result(component);
  NT collapse_time;
//
// XXX use is_squared in the counting zero length edges down below
  bool triangle_will_collapse = get_generic_collapse_time(time_now, determinant, collapse_time);
  if (triangle_will_collapse) {
    DBG(DBG_TRIANGLE_TIMING) << this << " collapse time is " << CGAL::to_double(collapse_time);
    const Point_2 p[3] = { vertex(0)->p_at(collapse_time),
                           vertex(1)->p_at(collapse_time),
                           vertex(2)->p_at(collapse_time) };
    const NT squared_lengths[3] = { CGAL::squared_distance(p[1],p[2]),
                                    CGAL::squared_distance(p[2],p[0]),
                                    CGAL::squared_distance(p[0],p[1]) };

    DBG(DBG_TRIANGLE_TIMING2) << this << " checking for edge lengths zero at time " << CGAL::to_double(collapse_time);
    bool is_zero[3];
    unsigned cnt_zero = 0;
    if ((is_zero[0] = (squared_lengths[0] == CORE_ZERO))) ++cnt_zero;
    if ((is_zero[1] = (squared_lengths[1] == CORE_ZERO))) ++cnt_zero;
    if (cnt_zero == 2) {
      assert_expensive_eq(squared_lengths[2], CORE_ZERO);
      cnt_zero = 3;
      is_zero[2] = true;
    } else if ((is_zero[2] = (squared_lengths[2] == CORE_ZERO))) ++cnt_zero;

    for (unsigned i=0; i<3; ++i) {
      DBG(DBG_TRIANGLE_TIMING2) << this << (is_zero[i] ? "  is zero" : "  is not zero.");
      DBG(DBG_TRIANGLE_TIMING2) << this << "  length: " << CGAL::to_double(squared_lengths[i]);
    }
    switch (cnt_zero) {
      case 3:
        result = CollapseSpec(component, CollapseType::TRIANGLE_COLLAPSE, collapse_time);
        break;
      case 1: {
          unsigned zero_idx = is_zero[0] ? 0 :
                              is_zero[1] ? 1 :
                                           2;
          assert_expensive(squared_lengths[cw(zero_idx)] == squared_lengths[ccw(zero_idx)]);
          result = CollapseSpec(component, CollapseType::SPOKE_COLLAPSE, collapse_time, zero_idx);
        }
        break;
      default:
        assert(cnt_zero == 0);
        {
          DBG(DBG_TRIANGLE_TIMING2) << this << " sorting lengths.";
          unsigned i0,i1,i2;
          std::tie(i0,i1,i2) = indirect_sort_3(squared_lengths);

          DBG(DBG_TRIANGLE_TIMING2) << this << " edge at collapse time is " << i0 << ".  length: " << CGAL::to_double(squared_lengths[i0]);
          DBG(DBG_TRIANGLE_TIMING2) << this << " edge at collapse time is " << i1 << ".  length: " << CGAL::to_double(squared_lengths[i1]);
          DBG(DBG_TRIANGLE_TIMING2) << this << " edge at collapse time is " << i2 << ".  length: " << CGAL::to_double(squared_lengths[i2]);

          assert(squared_lengths[i1] < squared_lengths[i2]);
          if (determinant.degree() == 0) {
            DBG(DBG_TRIANGLE_TIMING2) << this << " As determinant has degree zero, use current time as collapse time";
            collapse_time = time_now;
          };
          result = CollapseSpec(component, CollapseType::VERTEX_MOVES_OVER_SPOKE, collapse_time, i2, squared_lengths[i2]);
        }
        break;
    }
  } else {
    result = CollapseSpec(component, CollapseType::NEVER);
  }

  DBG(DBG_TRIANGLE) << this << " returning " << result;
  DBG_FUNC_END(DBG_TRIANGLE);
  return result;
} // }}}


/** only called for unconstrained triangles */
CollapseSpec
KineticTriangle::
compute_flip_event(const NT& time_now, const Polynomial_1& determinant) const { // {{{
  DBG_FUNC_BEGIN(DBG_TRIANGLE);
  DBG(DBG_TRIANGLE) << this;

  assert(!is_constrained(0) && !is_constrained(1) && !is_constrained(2));
  CollapseSpec result(component);

  bool could_flip = vertices[0]->is_reflex_or_straight() ||
                    vertices[1]->is_reflex_or_straight() ||
                    vertices[2]->is_reflex_or_straight();
  if (!could_flip) {
    result = event_that_will_not_happen(component, time_now, determinant);
  } else {
    result = get_generic_collapse(time_now, determinant);
    switch (result.type()) {
      case CollapseType::NEVER:
        break;
      case CollapseType::TRIANGLE_COLLAPSE:
      case CollapseType::SPOKE_COLLAPSE:
        LOG(INFO) << "compute_flip_event() found a triangle/spoke collapse: " << result;
        break;
      case CollapseType::VERTEX_MOVES_OVER_SPOKE:
        assert(!wavefronts[result.relevant_edge()]);
        if (vertices[result.relevant_edge()]->is_reflex_or_straight()) {
          // We are good, the vertex is reflex and the edge a spoke.
        } else {
          // The vertex is not reflex (or at least collinear), and therefore this is an event that
          // should never actually happen because we rebuild things until then.
          #ifdef DEBUG_COLLAPSE_TIMES
            result = CollapseSpec(component, CollapseType::INVALID_EVENT, result.time());
          #else
            result = CollapseSpec(component, CollapseType::NEVER);
          #endif
        }
        break;
      case CollapseType::FACE_HAS_INFINITELY_FAST_VERTEX_OPPOSING:
      case CollapseType::FACE_HAS_INFINITELY_FAST_VERTEX_WEIGHTED:
      case CollapseType::SPLIT_OR_FLIP_REFINE:
      case CollapseType::CONSTRAINT_COLLAPSE:
      case CollapseType::UNDEFINED:
      case CollapseType::CCW_VERTEX_LEAVES_CH:
      case CollapseType::INVALID_EVENT:
      default: {
        CANNOTHAPPEN_MSG << "Unexpected result from get_generic_collapse: " << result;
        assert(false);
        abort();
      }
    }
  }

  DBG(DBG_TRIANGLE) << this << " returning " << result;
  DBG_FUNC_END(DBG_TRIANGLE);
  return result;
} // }}}

/** Compute the collapse spec for a bounded triangle with 3 contraints.
 *
 * Since all 3 edges are constrained, this can only be a triangle
 * collapse.  This happens when all 3 edges collapse at the same time.
 */
CollapseSpec
KineticTriangle::
compute_collapse_bounded_constrained_3(const NT& time_now) const { // {{{
  DBG_FUNC_BEGIN(DBG_TRIANGLE);
  DBG(DBG_TRIANGLE) << this;

  CollapseSpec candidate(wavefronts[0]->get_collapse(component, time_now, 0));
  for (unsigned i=1; i<3; ++i) {
    assert(candidate == wavefronts[i]->get_collapse(component, time_now, i));
  };
  assert(candidate.type() == CollapseType::CONSTRAINT_COLLAPSE);
  CollapseSpec result(component, CollapseType::TRIANGLE_COLLAPSE, candidate.time());

  DBG(DBG_TRIANGLE) << "returning " << result;
  DBG_FUNC_END(DBG_TRIANGLE);
  return result;
} // }}}

/** Compute the collapse spec for a bounded triangle with 2 contraints.
 *
 * Each (constrained) edge collapse witnesses the vanishing of the triangle
 * and thus one of the roots of the triangle's determinant.
 *
 * If they collapse at the same time, this is a triangle collapse.  If not,
 * then this is an edge event.
 */
CollapseSpec
KineticTriangle::
compute_collapse_bounded_constrained_2(const NT& time_now) const { // {{{
  DBG_FUNC_BEGIN(DBG_TRIANGLE);
  DBG(DBG_TRIANGLE) << this;
  CollapseSpec result(component);

  unsigned c1_idx = wavefronts[0] ? 0 : 1;
  unsigned c2_idx = wavefronts[2] ? 2 : 1;
  assert(c1_idx != c2_idx);
  assert(wavefronts[c1_idx]);
  assert(wavefronts[c2_idx]);

  DBG(DBG_TRIANGLE) << "v0: " << vertices[0]->details();
  DBG(DBG_TRIANGLE) << "v1: " << vertices[1]->details();
  DBG(DBG_TRIANGLE) << "v2: " << vertices[2]->details();
  DBG(DBG_TRIANGLE) << "wavefront idx 1: " << c1_idx;
  DBG(DBG_TRIANGLE) << "wavefront idx 2: " << c2_idx;
  DBG(DBG_TRIANGLE) << "wavefront 1: " << *wavefronts[c1_idx];
  DBG(DBG_TRIANGLE) << "wavefront 2: " << *wavefronts[c2_idx];
  CollapseSpec c1(wavefronts[c1_idx]->get_collapse(component, time_now, c1_idx));
  CollapseSpec c2(wavefronts[c2_idx]->get_collapse(component, time_now, c2_idx));
  assert(c1.type() == CollapseType::CONSTRAINT_COLLAPSE || c1.type() == CollapseType::NEVER);
  assert(c2.type() == CollapseType::CONSTRAINT_COLLAPSE || c2.type() == CollapseType::NEVER);
  DBG(DBG_TRIANGLE) << "constraint collapse 1: " << c1;
  DBG(DBG_TRIANGLE) << "constraint collapse 2: " << c2;
  if (c1.type() == CollapseType::NEVER) {
    result = c2;
  } else if (c1.type() == CollapseType::NEVER) {
    result = c1;
  } else if (c1 == c2) { /* both constraints collapse at this time. */
    result = CollapseSpec(component, CollapseType::TRIANGLE_COLLAPSE, c1.time());
  } else {
    result = std::min(c1, c2);
  }

  DBG(DBG_TRIANGLE) << "returning " << result;
  DBG_FUNC_END(DBG_TRIANGLE);
  return result;
} // }}}



/** check if we like a specific collapse as an event.
 *
 * This checkfs whether the collapse at time is really the next instance
 * that the triangle collapses.
 *
 * The area of the triangle as a function of time is proportional to the
 * determinant and is a quadratic in time.  Together with the sign of the
 * leading coefficient of the determinant we can evaluate the derivative of the
 * determinant at time t to see whether this is the next time the determinant
 * vanishes.
 *
 * Unchecked precondition: right *now* (where we try to find the next event time),
 * the determinant of the triangle is not negative.  That is, it's a valid (or
 * degenerate) triangle.
 *
 * If the leading coefficient of det is
 *  - negative, then this means one triangle collapse was in the past already,
 *    and any event we found must be a real one.
 *  - positive, then we are either before or after the time when the area is
 *    negative.
 *    In such cases, we never want the second time (since it'd mean we came
 *    from an invalid triangulation to begin with), only the first.  So, to
 *    verify if the collapse time is the first or the second event, we
 *    look at the sign of the determinant's derivative evaluated at t.
 *    If it's negative, the collapse is the first instance of the triangle
 *    collapsing, otherwise, if the derivative at t is positive, the collapse
 *    is the second instance that the triangle collapses and we'll have
 *    to look for a real event prior.
 *    If the derivative is zero, then this is the only event this triangle
 *    will ever see.  Handle it.
 */
bool
KineticTriangle::
accept_collapse_bounded_constrained_1(const NT& collapse_time, const Polynomial_1& determinant, bool collapse_time_is_edge_collapse) { // {{{
  DBG_FUNC_BEGIN(DBG_TRIANGLE);
  bool result;

  assert(determinant.degree() == 2);
  auto determinant_sign = determinant.sign();
  assert(determinant_sign != CGAL::ZERO);

  if (determinant_sign == CGAL::NEGATIVE) {
    DBG(DBG_TRIANGLE) << "Sign is negative, event must be good.  (One collapse was in the past, there only is one more.)";
    result = true;
  } else {
    assert(determinant_sign == CGAL::POSITIVE);
    DBG(DBG_TRIANGLE) << "Sign is positive, checking if we got the first or second event.";

    Polynomial_1 derivative = differentiate(determinant);
    const NT derivative_at_collapse = evaluate(derivative, collapse_time);
    DBG(DBG_TRIANGLE) << "derivative(t): " << CGAL::to_double(derivative_at_collapse);

    switch (CGAL::sign(derivative_at_collapse)) {
      case CGAL::ZERO:
        DBG(DBG_TRIANGLE) << "Derivative is zero.  If an edge collapses right now, then either the triangle collapses entirely, or the 3rd vertex moves over our supporting line right now.  Of course it could also just be that the vertices are collinear exactly once.";
        // DBG(DBG_TRIANGLE) << "At any rate, since the sign of the determinant is positive, the triangle has positive area after this event and we do not need to do anything here.";
        DBG(DBG_TRIANGLE) << "At any rate, this is the only event the triangle will ever see.  Handle it.";
        result = true;
        break;
      case CGAL::NEGATIVE:
        DBG(DBG_TRIANGLE) << "Derivative is negative.  This is the first time the triangle collapses.  We want it.";
        result = true;
        break;
      case CGAL::POSITIVE:
        DBG(DBG_TRIANGLE) << "Derivative is positive.  This is the second time the triangle collapses.  This triangle MUST change before the first time it collapses.";
        result = false;
        break;
      default:
        CANNOTHAPPEN_MSG << "Fell through switch which should cover all cases.";
        abort();
    }
  }

  DBG(DBG_TRIANGLE) << "returning " << result;
  DBG_FUNC_END(DBG_TRIANGLE);
  return result;
} // }}}

/** Compute the collapse spec for a bounded triangle with 1 contraint.
 *
 */
CollapseSpec
KineticTriangle::
compute_collapse_bounded_constrained_1(const NT& time_now) const { // {{{
  DBG_FUNC_BEGIN(DBG_TRIANGLE);
  DBG(DBG_TRIANGLE) << this;
  DBG(DBG_TRIANGLE) << " v0 " << vertex(0)->details();
  DBG(DBG_TRIANGLE) << " v1 " << vertex(1)->details();
  DBG(DBG_TRIANGLE) << " v2 " << vertex(2)->details();
  CollapseSpec result(component);


  // XXX only compute the determinant if we need it
  //
  const Polynomial_1 determinant(compute_determinant_from_vertices(vertex(0), vertex(1), vertex(2)));
  DBG(DBG_TRIANGLE) << "det(time) " << CGAL::to_double(evaluate(determinant, time_now));
  assert_expensive(evaluate(determinant, time_now) >= CORE_ZERO);

  unsigned c_idx = !!wavefronts[0] ? 0 :
                   !!wavefronts[1] ? 1 :
                                     2;
  const WavefrontEdge * const wf = wavefronts[c_idx];
  if (wf->parallel_endpoints(time_now)) {
    const EdgeCollapseSpec& edge_collapse = wf->get_edge_collapse(time_now);
    DBG(DBG_TRIANGLE) << "Edge endpoints are parrallel;  collapse is " << edge_collapse << "; det degree is " << determinant.degree();
    assert(edge_collapse.type() != EdgeCollapseType::ALWAYS);
    if (determinant.degree() == 1) {
      result = compute_split_or_flip_event_bounded_constrained_1(time_now, c_idx, determinant);
    } else {
      result = CollapseSpec(component, CollapseType::NEVER);
    }
  } else {
    CollapseSpec candidate(wf->get_collapse(component, time_now, c_idx));
    DBG(DBG_TRIANGLE) << "Edge collapse is " << candidate << "; determinant degree is " << determinant.degree();
    assert(candidate.type() == CollapseType::CONSTRAINT_COLLAPSE || candidate.type() == CollapseType::NEVER);

    if (determinant.degree() == 2) { // The edge could collapse, or we could flip/split
      bool have_collapse = candidate.type() == CollapseType::CONSTRAINT_COLLAPSE && accept_collapse_bounded_constrained_1(candidate.time(), determinant, true);

      if (have_collapse) {
        DBG(DBG_TRIANGLE) << "We like the edge collapse.";
        result = candidate;
      } else {
        DBG(DBG_TRIANGLE) << "We did not like the edge collapse.  Hunt for the real event.";
        result = compute_split_or_flip_event_bounded_constrained_1(time_now, c_idx, determinant);
      }
    } else {
      assert(determinant.degree() <= 1);
      if (candidate.type() == CollapseType::NEVER) {
        DBG(DBG_TRIANGLE) << "Determinant: " << determinant;
        DBG(DBG_TRIANGLE) << "Determinant degree < 2 and non-parallel endpoints of the constraint which will never collapse (so would have collapsed in the past).";
        result = compute_split_or_flip_event_bounded_constrained_1(time_now, c_idx, determinant);
      } else {
        assert(candidate.type() == CollapseType::CONSTRAINT_COLLAPSE);
        DBG(DBG_TRIANGLE) << "Determinant degree < 2 and non-parallel endpoints of the constraint which will collapse.  We will use the constraint collapse.";
        result = candidate;
      }
    }
  }

  DBG(DBG_TRIANGLE) << "returning " << result;
  DBG_FUNC_END(DBG_TRIANGLE);
  return result;
} // }}}

/** Compute the collapse spec for a bounded triangle with 0 contraints.
 *
 * Such a triangle can either see a "meet" event, where two non-incident
 * vertices become incident, or it can see a flip event where a reflex
 * vertex moves over a triangluation spoke.
  // XXX do meet events.

 */
CollapseSpec
KineticTriangle::
compute_collapse_bounded_constrained_0(const NT& time_now) const { // {{{
  DBG_FUNC_BEGIN(DBG_TRIANGLE);
  DBG(DBG_TRIANGLE) << this;
  CollapseSpec result(component);

  const Polynomial_1 determinant(compute_determinant_from_vertices(vertex(0), vertex(1), vertex(2)));
  result = compute_flip_event(time_now, determinant);

  DBG(DBG_TRIANGLE) << "returning " << result;
  DBG_FUNC_END(DBG_TRIANGLE);
  return result;
} // }}}

CollapseSpec
KineticTriangle::
compute_collapse_bounded(const NT& time_now) const { /// {{{
  DBG_FUNC_BEGIN(DBG_TRIANGLE);
  DBG(DBG_TRIANGLE) << this;
  CollapseSpec result(component);

  /** See notes on classification from 20180731 */
  unsigned num_wavefronts = !!wavefronts[0] + !!wavefronts[1] + !!wavefronts[2];
  DBG(DBG_TRIANGLE) << "Have " << num_wavefronts << " constrained edge(s).";
  switch (num_wavefronts) {
    case 3:
      result = compute_collapse_bounded_constrained_3(time_now);
      break;
    case 2:
      result = compute_collapse_bounded_constrained_2(time_now);
      break;
    case 1:
      result = compute_collapse_bounded_constrained_1(time_now);
      break;
    case 0:
      result = compute_collapse_bounded_constrained_0(time_now);
      break;
    default:
      CANNOTHAPPEN_MSG << "Invalid number of constrained edges: " << num_wavefronts;
      assert(false);
      abort();
  }
  assert(result.type() != CollapseType::UNDEFINED);

  DBG(DBG_TRIANGLE) << "returning " << result;
  DBG_FUNC_END(DBG_TRIANGLE);
  return result;
} // }}}


/* unbounded triangles witness two things:
 *   - collapse of their bounded spoke
 *   - one of their vertices leaving the CH boundary.
 *     (the vertex CCW from the infinite vertex, i.e.,
 *      the first vertex when traversing the convex hull in CW order.)
 *      let that vertex be v.
 */
CollapseSpec
KineticTriangle::
compute_collapse_unbounded(const NT& time_now) const { // {{{
  DBG_FUNC_BEGIN(DBG_TRIANGLE);
  DBG(DBG_TRIANGLE) << this;

  CollapseSpec result(component);
  CollapseSpec edge_collapse(component);

  assert(unbounded());
  assert(has_vertex_infinite_speed() == InfiniteSpeedType::NONE);
  unsigned idx = infinite_vertex_idx();
  if (is_constrained(idx)) {
    edge_collapse = wavefronts[idx]->get_collapse(component, time_now, idx);
    DBG(DBG_TRIANGLE_TIMING) << "   Constraint will collapse at " << edge_collapse;
  } else {
    edge_collapse = CollapseSpec(component, CollapseType::NEVER);
    DBG(DBG_TRIANGLE_TIMING) << "   not constraint.";
  }

  KineticTriangle *n = neighbor(cw(idx));
  assert(n->unbounded());
  unsigned nidx = n->infinite_vertex_idx();
  assert(n->neighbor(ccw(nidx)) == this);
  assert(vertex(ccw(idx)) == n->vertex(cw(nidx)));

  // v is the (finite) vertex common to this and n.
  if (is_constrained(idx) && n->is_constrained(nidx)) {
    DBG(DBG_TRIANGLE_TIMING) << "both this and n are constraint -- v cannot leave the CH boundary.  Constraint collapse is the only thing that may happen.";
    result = edge_collapse;
  } else if (is_constrained(idx) || n->is_constrained(nidx)) {
    DBG(DBG_TRIANGLE_TIMING) << "Exactly one of this and n is constraint.";
    /* one of this and n is constraint.
     */
    CollapseSpec vertex_leaves_ch(component);
    const WavefrontSupportingLine *e;
    const WavefrontVertex *v;
    if (is_constrained(idx)) {
      e = wavefront(idx)->l().get();
      v = n->vertex(ccw(nidx));
    } else {
      e = n->wavefront(nidx)->l().get();
      v = vertex(cw(idx));
    };
    assert(v);
    assert(e);

    CGAL::Sign edge_is_faster = edge_is_faster_than_vertex(*v, *e);
    if (edge_is_faster == CGAL::ZERO) {
      DBG(DBG_TRIANGLE_TIMING) << "   Edge " << e << " is just as fast as the vertex " << v;

      NT collapse_time;
      VertexOnSupportingLineType vertex_on_line_type;
      std::tie(collapse_time, vertex_on_line_type) = get_time_vertex_on_supporting_line(*v, *e);
      if (vertex_on_line_type == VertexOnSupportingLineType::NEVER) {
        DBG(DBG_TRIANGLE_TIMING) << "     but v is not on the supporting line of e";
      } else {
        assert(vertex_on_line_type == VertexOnSupportingLineType::ALWAYS);
        DBG(DBG_TRIANGLE_TIMING) << "     and they are collinear.  And while that is the case, v cannot leave the CH.";
        /* v moving into us is witnessed by the neighboring triangle */
      }
      vertex_leaves_ch = CollapseSpec(component, CollapseType::NEVER);
    } else if (edge_is_faster == CGAL::POSITIVE) {
      DBG(DBG_TRIANGLE_TIMING) << "   Edge " << e << " is faster than vertex " << v << " - CCW of t will never leave the CH.";
      vertex_leaves_ch = CollapseSpec(component, CollapseType::NEVER);
    } else {
      DBG(DBG_TRIANGLE_TIMING) << "   Edge " << e << " is slow than vertex " << v << " - CCW of t will leave the CH.";

      NT collapse_time;
      VertexOnSupportingLineType vertex_on_line_type;
      std::tie(collapse_time, vertex_on_line_type) = get_time_vertex_on_supporting_line(*v, *e);
      assert(vertex_on_line_type == VertexOnSupportingLineType::ONCE);

      DBG(DBG_TRIANGLE_TIMING) << "   * vertex will move onto supporting line at " << CGAL::to_double(collapse_time);
      DBG(DBG_TRIANGLE_TIMING) << "   * now                                    t " << CGAL::to_double(time_now);
      assert_expensive_ge(collapse_time, time_now);
      vertex_leaves_ch = CollapseSpec(component, CollapseType::CCW_VERTEX_LEAVES_CH, collapse_time, idx);
    }

    DBG(DBG_TRIANGLE_TIMING) << "  edge_collapse   : " << edge_collapse;
    DBG(DBG_TRIANGLE_TIMING) << "  vertex_leaves_ch: " << vertex_leaves_ch;
    result = std::min(vertex_leaves_ch, edge_collapse);
  } else {
    DBG(DBG_TRIANGLE_TIMING) << "Neither this nor n are constraint -- a vertex leaving the CH is the only thing that can happen.";
    assert(edge_collapse.type() == CollapseType::NEVER);
    /* we need to do set up a determinant and solve for zeroes.
     */
    const WavefrontVertex * const u = vertex(cw (idx));
    const WavefrontVertex * const v = vertex(ccw(idx));
    const WavefrontVertex * const V = n->vertex(cw (nidx));
    const WavefrontVertex * const w = n->vertex(ccw(nidx));
    assert(v==V);
    assert_expensive(CGAL::orientation(u->p_at(time_now),
                                       v->p_at(time_now),
                                       w->p_at(time_now)) != CGAL::RIGHT_TURN);
    if (u->velocity == v->velocity && v->velocity == w->velocity) {
      DBG(DBG_TRIANGLE_TIMING) << "   * all three vertices have the same velocity ";
      if (u->pos_zero == v->pos_zero || v->pos_zero == w->pos_zero) {
        DBG(DBG_TRIANGLE_TIMING) << "   * at least two vertices are incident. ";
        NOTIMPL_MSG << "Incident vertices case";
        assert(false); //untested branch is_constrained_both
        abort();
      } else {
        DBG(DBG_TRIANGLE_TIMING) << "   * Three parallel vertices on the ch";
        result = CollapseSpec(component, CollapseType::NEVER);
      }
    } else {
      const Polynomial_1 determinant(compute_determinant_from_vertices(u, v, w));
      NT time;
      if (get_generic_collapse_time(time_now, determinant, time)) {
        DBG(DBG_TRIANGLE_TIMING) << "   * CCW will leave CH in unconstrained situation at time " << CGAL::to_double(time);
        if (time == time_now) {
          LOG(WARNING) << "Rarely exercised code path: unconstraint case with triangle leaving right now.";
        }
        result = CollapseSpec(component, CollapseType::CCW_VERTEX_LEAVES_CH, time, idx);
      } else {
        DBG(DBG_TRIANGLE_TIMING) << "   * CCW will not leave CH in unconstrained situation";
        result = CollapseSpec(component, CollapseType::NEVER);
      }
    }
  }

  DBG(DBG_TRIANGLE) << this << " returning " << result;
  DBG_FUNC_END(DBG_TRIANGLE);
  return result;
} // }}}

void
KineticTriangle::
set_dead() { // {{{
  assert(!is_dead_);
  for (unsigned i=0; i<3; ++i) {
    if (neighbors[i]) {
      assert(! neighbors[i]->has_neighbor(this));
    }
    if (wavefronts[i]) {
      assert(wavefronts[i]->incident_triangle() != this || wavefronts[i]->is_dead());
    }
  }
  is_dead_ = true;
} // }}}

void
KineticTriangle::
set_vertex(unsigned i, WavefrontVertex *v) { // {{{
  SRF_precondition(i < 3);
  assert(v);
  vertices[i] = v;
  if (is_constrained(cw(i))) {
    assert(wavefronts[cw(i)]);
    wavefronts[cw(i)]->set_wavefrontedge_vertex(0, v);
  }
  if (is_constrained(ccw(i))) {
    assert(wavefronts[ccw(i)]);
    wavefronts[ccw(i)]->set_wavefrontedge_vertex(1, v);
  }
  invalidate_collapse_spec();
} // }}}

void
KineticTriangle::
set_wavefront(unsigned idx, WavefrontEdge *e) { // {{{
  CGAL_precondition(idx < 3);
  assert(e);

  assert(wavefronts[idx] == NULL);
  assert(neighbors[idx]);
  assert(neighbors[idx]->is_dying());
  assert(e->incident_triangle() == neighbors[idx]);

  wavefronts[idx] = e;
  neighbors[idx] = NULL;
  e->set_incident_triangle(this);
} // }}}

void
KineticTriangle::
move_constraint_from(unsigned idx, KineticTriangle &src, unsigned src_idx) { /// {{{
  CGAL_precondition(idx < 3 && src_idx < 3);

  assert(src.is_dying());
  assert(!is_constrained(idx));
  assert(src.wavefronts[src_idx]->incident_triangle() == &src);
  wavefronts[idx] = src.wavefronts[src_idx];

  // we already need to share one vertex with the origin, which will go away.
  assert(wavefronts[idx]->vertex(0) == vertices[ccw(idx)] ||
         wavefronts[idx]->vertex(1) == vertices[cw (idx)]);
  assert(has_neighbor(&src));
  assert(idx == index(&src));

  wavefronts[idx]->set_wavefrontedge_vertex(0, vertices[ccw(idx)]);
  wavefronts[idx]->set_wavefrontedge_vertex(1, vertices[cw (idx)]);
  wavefronts[idx]->set_incident_triangle(this);

  src.wavefronts[src_idx] = NULL;
  invalidate_collapse_spec();
} // }}}

/** Flip edge <idx> of this triangle.
 *
 * Let t be v, v1, v2.  (where v is the vertex at idx),
 * and let our neighbor opposite v be o, v2, v1.
 *
 * Then after the flipping, this triangle will be v, o, v2,
 * and the neighbor will be o, v, v1.
 */
void
KineticTriangle::
do_raw_flip_inner(unsigned idx) { // {{{
  DBG_FUNC_BEGIN(DBG_TRIANGLE | DBG_TRIANGLE_FLIP);
  DBG(DBG_TRIANGLE | DBG_TRIANGLE_FLIP) << this;
  DBG(DBG_TRIANGLE_FLIP) << this << " * flipping at idx " << idx;
  assert(!is_constrained(idx));
  KineticTriangle* n = neighbors[idx];
  int nidx = n->index(this);
  DBG(DBG_TRIANGLE_FLIP) << "   neighbor " << n << " with nidx " << nidx;

  WavefrontVertex * const v = vertices[idx];
  WavefrontVertex * const v1 = vertices[ccw(idx)];
  WavefrontVertex * const v2 = vertices[cw (idx)];
  WavefrontVertex * const o = n->vertices[nidx];
  assert(v1 == n->vertex(cw (nidx)));
  assert(v2 == n->vertex(ccw(nidx)));

  // set new triangles
  vertices[ccw(idx )] = o;
  n->vertices[ccw(nidx)] = v;

  DBG(DBG_TRIANGLE_FLIP) << "   * t  " << this;
  DBG(DBG_TRIANGLE_FLIP) << "     n  " << n;

  DBG(DBG_TRIANGLE_FLIP) << "     na " << n->neighbors [cw(idx)];
  DBG(DBG_TRIANGLE_FLIP) << "     nb " << neighbors [cw(idx)];

  // fix neighborhood relations and fix/move constraints
  // - neighbors of t and n that see their neighbor
  //   change from n to t or t to n.
  neighbors [idx] = n->neighbors [cw(nidx)];
  wavefronts[idx] = n->wavefronts[cw(nidx)];
  n->neighbors [nidx] = neighbors [cw(idx)];
  n->wavefronts[nidx] = wavefronts[cw(idx)];

  // - pair up t and n
  n->neighbors [cw(nidx)] = this;
  n->wavefronts[cw(nidx)] = NULL;
  neighbors [cw(idx)] = n;
  wavefronts[cw(idx)] = NULL;

  DBG(DBG_TRIANGLE_FLIP) << "   * t  " << this;
  DBG(DBG_TRIANGLE_FLIP) << "     n  " << n;

  assert(!!neighbors[idx] == !wavefronts[idx]);
  assert(!!n->neighbors[nidx] == !n->wavefronts[nidx]);

  if (is_constrained(idx)) {
    wavefronts[idx]->set_incident_triangle(this);
  } else {
    auto i = neighbors[idx]->index(n);
    neighbors[idx]->neighbors[i] = this;
  }
  if (n->wavefronts[nidx]) {
    n->wavefronts[nidx]->set_incident_triangle(n);
  } else {
    auto i = n->neighbors[nidx]->index(this);
    n->neighbors[nidx]->neighbors[i] = n;
  }

  DBG_FUNC_END(DBG_TRIANGLE | DBG_TRIANGLE_FLIP);
} // }}}

/** Flip edge <idx> of this triangle.
 *
 * Run do_raw_flip_inner() and asserts validity after
 * and invalidates collapse specs.
 */
void
KineticTriangle::
do_raw_flip(unsigned idx) { // {{{
  DBG_FUNC_BEGIN(DBG_TRIANGLE | DBG_TRIANGLE_FLIP);
  KineticTriangle* n = neighbors[idx];

  do_raw_flip_inner(idx);

  assert_valid();
  n->assert_valid();

  invalidate_collapse_spec();
  n->invalidate_collapse_spec();

  DBG_FUNC_END(DBG_TRIANGLE | DBG_TRIANGLE_FLIP);
} // }}}

std::ostream& operator<<(std::ostream& os, const KineticTriangle * const kt) { // {{{
  if (kt) {
    char sep[3] = {',', ',', ';'};
    char sep2[3] = {',', ',', ';'};

    os << kt->get_name()
       << " (@"
       << (void*)kt << "; ";
    for (int i=0; i<3; ++i) {
      os << kt->vertices[i] << sep[i];
    }
    os << " ";
    for (int i=0; i<3; ++i) {
      if (kt->neighbors[i]) {
        os << kt->neighbors[i]->get_name() << sep2[i];
      } else {
        os << "*" << sep[i];
      };
    };
    os << " ";
    for (int i=0; i<3; ++i) {
      if (kt->wavefronts[i]) {
        os << *kt->wavefronts[i] << sep2[i];
      } else {
        os << "*" << sep2[i];
      };
    };
    os << " c" << kt->component << ")";
  } else {
    os << "kt*";
  }
  return os;
} // }}}

std::ostream& operator<<(std::ostream& os, const KineticTriangle::VertexOnSupportingLineType a) { // {{{
  switch (a) {
    case KineticTriangle::VertexOnSupportingLineType::ONCE:
      os << "ONCE";
      break;
    case KineticTriangle::VertexOnSupportingLineType::ALWAYS:
      os << "ALWAYS";
      break;
    case KineticTriangle::VertexOnSupportingLineType::NEVER:
      os << "NEVER";
      break;
  }
  return os;
} // }}}

// vim:set fdm=marker:
