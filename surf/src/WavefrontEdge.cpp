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
#include "WavefrontEdge.h"
#include "WavefrontVertex.h"
#include "KineticTriangle.h"

unsigned WavefrontEdge::wavefront_edge_ctr = 0;

std::ostream&
operator<<(std::ostream& os, const WavefrontEdge& e) {
  os << "wfe#"
    << e.id
    << "("
    << e.vertices[0] << ","
    << e.vertices[1] << ")";
  return os;
}

/** returns when this edge will collapse.
 *
 * If the two vertices are parallel or moving away from one another,
 * they will NEVER collapse.  Even if the two end-points are
 * conincident right now but are moving away from another, we will consider
 * this as not collapsing.
 *
 * Otherwise, they will collapse at some point in time.  Since
 * we are asking, we assume (and we assert() during debugging),
 * that this will be in the future (or now).
 */
EdgeCollapseSpec
WavefrontEdge::
compute_collapse(const NT& time_now) const {
  DBG_FUNC_BEGIN(DBG_KT);
  DBG(DBG_KT) << "Computing edge collapse time for " << *this;

  EdgeCollapseSpec res;
  WavefrontVertex* wfv0 = vertices[0];
  WavefrontVertex* wfv1 = vertices[1];
  assert(wfv0);
  assert(wfv1);
  const Vector_2& v0 = wfv0->velocity;
  const Vector_2& v1 = wfv1->velocity;

  DBG(DBG_KT) << "v0" << CGAL_vector(v0);
  DBG(DBG_KT) << "v1" << CGAL_vector(v1);
  CGAL::Orientation o(CGAL::orientation(v0,v1));

  if (o != CGAL::LEFT_TURN) {
    /* If the two wavefront vertices move away from each other
     * or in parallel, this edge will never collapse.
     */
    if (o == CGAL::RIGHT_TURN) {
      DBG(DBG_KT) << "Orientation is right turn";
      // let's consider two points that are identical right now but moving away from another as not collapsing.
      res = EdgeCollapseSpec(EdgeCollapseType::PAST);
    } else {
      DBG(DBG_KT) << "Orientation is collinear";
      assert(o == CGAL::COLLINEAR);

      /* Previously we computed the distance at time_now.  I wonder why.
       * If we then claim that the edge is always collapsing, then it should
       * suffice to compute the distance at t=0. */
      const NT sqdist = CGAL::squared_distance(wfv0->pos_zero, wfv1->pos_zero);
      DBG(DBG_KT) << "sqdist zero: " << CGAL::to_double(sqdist);

      {
        const Point_2 p0(wfv0->p_at(time_now));
        const Point_2 p1(wfv1->p_at(time_now));
        const NT sqdistnow = CGAL::to_double(CGAL::squared_distance(p0, p1));
        DBG(DBG_KT) << "sqdist now : " << sqdistnow;
        if (sqdist == CORE_ZERO) {
          assert_expensive_eq(sqdist, sqdistnow);
        } else {
          assert(sqdistnow > CORE_ZERO);
        }
      }

      if (sqdist == CORE_ZERO) {
        DBG(DBG_KT) << "Distance is zero now.";
        res = EdgeCollapseSpec(EdgeCollapseType::ALWAYS, time_now);
      } else {
        DBG(DBG_KT) << "Distance is not zero.";
        res = EdgeCollapseSpec(EdgeCollapseType::NEVER);
      }
    }
  } else {
    DBG(DBG_KT) << "Orientation is left turn";
  /* Note that, by construction, if you start on the wavefront edge, go out v0,
   * and go back v1, you end up on the wavefrong edge again.  Or, in other words,
   * v0-v1 is collinear with the direction of the wavefront edge e.
   *
   * Now, we want to know when e=AB collapses.  So we can restrict ourselves to
   * consider the projection of A+t*v0 and B+t*v1 to e itself.  Once they meet,
   * the edge collapses.  Equivalently, once the length of projected t*v0 and
   * projected t*v1 equals the length of e, the edge collapses.
   *
   * Let d be the vector AB (i.e., B-A).  The dot product v0.d is the length of
   * the projected v0 times the length of d.  So v0.d/|d| is the length of the
   * projected v0.  Likewise, v1.d/|d| is the length of the projected v1.
   * Thus e will collapse at time t := |d| / ( v0.d/|d| - v1.d/|d| ) ==
   * == |d|^2 / ( v0.d - v1.d ) == |d|^2 / ( (v0 - v1).d) ==
   * == d.d /  ( (v0 - v1).d).
   *
   * Isn't that interesting?  Note how we compare the length of d projected
   * onto d with the length of (v0 - v1) projected onto d?  Remember that, as
   * previously mentioned, by construction, (v0 - v1) is in the same direction
   * as d.  Thus, this quotient will have the same value regardless of
   * which line we project both d and (v0 - v1) -- as long as the line is not
   * orthogonal to (v0 - v1).  So we might just as well project onto (1,0) and
   * only consider their x-coordinates (if d is not exactly vertical).
   * So t == d_x / (v0_x - v1_x)
   *
   * This also makes sense when looking at it another way.  Consider the points
   * of the projection of A+t*v0 and B+t*v1 onto e.  Since they are on e, they
   * will become incident when and only when their x-coordinates is the same.
   */
    NT edge_delta;
    NT wfvs_delta;
    if (! supporting_line->l.is_vertical()) {
      edge_delta = wfv1->pos_zero.x() - wfv0->pos_zero.x();
      wfvs_delta = v0.x() - v1.x();
    } else { /* l is vertical, do the same with y-coordinates */
      edge_delta = wfv1->pos_zero.y() - wfv0->pos_zero.y();
      wfvs_delta = v0.y() - v1.y();
    }

    assert(edge_delta != 0);
    assert(wfvs_delta != 0);
    NT time = edge_delta / wfvs_delta;
    DBG(DBG_KT) << "future edge collapse: " << CGAL::to_double(time);
    DBG(DBG_KT) << "time_now            : " << CGAL::to_double(time_now);
    assert_ge(time, time_now);
    res = EdgeCollapseSpec(EdgeCollapseType::FUTURE, time);
  }
  DBG(DBG_KT) << "returning " << res;
  DBG_FUNC_END(DBG_KT);
  return res;
}

#ifndef NDEBUG
void
WavefrontEdge::
assert_edge_sane(int collapsing_edge) const {
  assert(0 <= collapsing_edge && collapsing_edge < 3);
  assert(incident_triangle_);
  assert(incident_triangle_->wavefront(collapsing_edge) == this);
  assert(vertices[0]);
  assert(vertices[1]);
}
#endif

void
WavefrontEdge::
set_incident_triangle(KineticTriangle* incident_triangle) {
  assert(!is_dead_);
  assert(incident_triangle->has_wavefront(this));
  unsigned idx = incident_triangle->index(this);

  incident_triangle_ = incident_triangle;

  assert(incident_triangle->vertex(KineticTriangle::ccw(idx)) == vertices[0]);
  assert(incident_triangle->vertex(KineticTriangle::cw (idx)) == vertices[1]);
  invalidate_collapse_spec();
}

/** Duplicate this edge in the course of a split event.
 *
 * This is just a simple helper that creates two copies of this edge and marks
 * the original as dead.
 *
 * It is the job of the caller (KineticTriangulation) to then give us new vertices.
 */
WavefrontEdge::EdgePtrPair
WavefrontEdge::
split(WavefrontEdgeList& wavefront_edges) {
  assert(vertices[0]);
  assert(vertices[1]);
  assert(vertices[0]->incident_wavefront_edge(1) == this);
  assert(vertices[1]->incident_wavefront_edge(0) == this);
  set_dead();

  wavefront_edges.emplace_back( WavefrontEdge(vertices[0], NULL, supporting_line, incident_triangle_, skeleton_face) );
  auto pea = &wavefront_edges.back();
  wavefront_edges.emplace_back( WavefrontEdge(NULL, vertices[1], supporting_line, incident_triangle_, skeleton_face) );
  auto peb = &wavefront_edges.back();

  return EdgePtrPair(pea, peb);
}
