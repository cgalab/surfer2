/**  surfer2 -- a straight skeleton implementation
 *
 *  Copyright 2018, 2019 Peter Palfrader
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
#include "WavefrontVertex.h"
#include "KineticTriangle.h"

DEBUG_DECL(
unsigned WavefrontVertex::kvctr = 0;
)

WavefrontVertex::
WavefrontVertex(
  const Point_2& p_pos_zero,
  const Point_2& p_pos_start,
  const NT& p_time_start,
  const WavefrontEdge * const a,
  const WavefrontEdge * const b,
  bool p_is_initial,
  bool p_is_beveling,
  bool p_is_infinite)
  :
  #ifndef SURF_NDEBUG
    id(kvctr++),
  #endif
    pos_zero(p_pos_zero)
  , pos_start(p_pos_start)
  , time_start(p_time_start)
  , incident_wavefront_edges {a, b}
  , angle(a && b ? CGAL::orientation(a->l()->l.to_vector(), b->l()->l.to_vector()) : STRAIGHT)
  , is_initial(p_is_initial)
  , is_beveling(p_is_beveling)
  , is_infinite(p_is_infinite)
  , infinite_speed(get_infinite_speed_type(a, b, angle))
  , velocity( (a && b && (infinite_speed == InfiniteSpeedType::NONE)) ? compute_velocity(pos_zero, *a->l(), *b->l(), angle) : CGAL::NULL_VECTOR)
  , px_( (infinite_speed != InfiniteSpeedType::NONE) ? Polynomial_1(0) : Polynomial_1(pos_zero.x(), velocity.x()) )
  , py_( (infinite_speed != InfiniteSpeedType::NONE) ? Polynomial_1(0) : Polynomial_1(pos_zero.y(), velocity.y()) )
  , skeleton_dcel_halfedge_ { NULL, NULL }
  , next_vertex_ { NULL, NULL }
  , prev_vertex_ { NULL, NULL }

{
  assert(!!a == !!b);
}

InfiniteSpeedType
WavefrontVertex::
get_infinite_speed_type(const WavefrontEdge * const a, const WavefrontEdge * const b, const VertexAngle& angle) {
  if (a && b && angle == STRAIGHT) {
    if (CGAL::orientation(a->l()->l.to_vector(), b->l()->l.to_vector().perpendicular(CGAL::CLOCKWISE)) == CGAL::LEFT_TURN) {
      return InfiniteSpeedType::OPPOSING;
    } else if (a->l()->weight != b->l()->weight) {
      return InfiniteSpeedType::WEIGHTED;
    } else {
      assert(a->l()->normal == b->l()->normal);
      return InfiniteSpeedType::NONE;
    }
  } else {
    return InfiniteSpeedType::NONE;
  }
}

std::tuple<WavefrontVertex::LineIntersectionType, Point_2>
WavefrontVertex::
compute_intersection(const Line_2& a, const Line_2& b) {
  const auto intersect0 = intersection(a, b);
  if (intersect0) {
    if (const Line_2* s = boost::get<Line_2>(&*intersect0)) {
      return std::make_tuple(LineIntersectionType::ALL, CGAL::ORIGIN);
    } else {
      const Point_2* p = boost::get<Point_2 >(&*intersect0);
      assert(p);
      return std::make_tuple(LineIntersectionType::ONE, *p);
    }
  } else {
    return std::make_tuple(LineIntersectionType::NONE, CGAL::ORIGIN);
  };
}

WavefrontVertex
WavefrontVertex::
make_vertex(
  const Point_2& pos,
  const NT& time,
  const WavefrontEdge* const a,
  const WavefrontEdge* const b,
  bool from_split
) {
  DBG_FUNC_BEGIN(DBG_KT);
  DBG(DBG_KT) << "a:" << *a << " " << CGAL_line(a->l()->l);
  DBG(DBG_KT) << "b:" << *b << " " << CGAL_line(b->l()->l);

  if (! from_split) {
    assert(a->vertex(1) && a->vertex(1)->has_stopped());
    assert(b->vertex(0) && b->vertex(0)->has_stopped());
  }

  Point_2 pos_zero;
  LineIntersectionType lit;
  std::tie(lit, pos_zero) = compute_intersection(a->l()->l, b->l()->l);
  switch (lit) {
    case LineIntersectionType::ALL:
      pos_zero = pos - time * compute_velocity(CGAL::ORIGIN, *a->l(), *b->l(), STRAIGHT);
      // fall through
    case LineIntersectionType::ONE:
      break;
    case LineIntersectionType::NONE:
      DBG(DBG_KT) << "No intersection at time 0 between supporting lines of wavefrontedges.  Parallel wavefronts crashing (or wavefronts of different speeds becoming collinear).";
      pos_zero = pos;
      break;
    default:
      CANNOTHAPPEN_MSG << "Fell through switch which should cover all cases.";
      assert(false);
      abort();
  }

  WavefrontVertex v = WavefrontVertex(pos_zero, pos, time, a, b);
  assert( (lit == LineIntersectionType::NONE) == (v.infinite_speed != InfiniteSpeedType::NONE));

  assert(v.p_at(time) == pos);
  DBG_FUNC_END(DBG_KT);
  return v;
}

Vector_2
WavefrontVertex::
compute_velocity(
  const Point_2& pos_zero,
  const WavefrontSupportingLine& a,
  const WavefrontSupportingLine& b,
  const VertexAngle angle) {
  Vector_2 result;

  if (angle != STRAIGHT) {
    const Line_2& la = a.line_at_one();
    const Line_2& lb = b.line_at_one();

    Point_2 intersect;
    LineIntersectionType lit;
    std::tie(lit, intersect) = compute_intersection(la, lb);
    if (lit != LineIntersectionType::ONE) {
      CANNOTHAPPEN_MSG << "No point intersection between WavefrontEmittingEdges at offset 1.  Bad.";
      assert(false);
      abort();
    }
    result = intersect - pos_zero;
  } else {
    DBG(DBG_KT) << "a:" << CGAL_vector(a.normal);
    DBG(DBG_KT) << "b:" << CGAL_vector(b.normal);

    if ( CGAL::orientation(a.l.to_vector(), b.l.to_vector().perpendicular(CGAL::CLOCKWISE)) == CGAL::RIGHT_TURN ) {
      /* They are in the same direction */
      if (a.normal == b.normal) {
        result = a.normal;
      } else {
        NOTIMPL_MSG << "collinear incident wavefront edges with different speeds.";
        assert(false);
        exit(1);
      }
    } else {
      CANNOTHAPPEN_MSG << "This is an infinitely fast vertex.  We should not be in compute_velocity().";
      assert(false);
      abort();
    }
  }
  return result;
}

std::string
WavefrontVertex::
details() const {
  std::ostringstream oss;
  DEBUG_STMT(oss << "kv" << id);
  oss << "(";
  if (is_infinite) {
    oss << "inf";
  } else {
    oss << "wf: " << *incident_wavefront_edges[0]
        << "; "   << *incident_wavefront_edges[1];
    oss << "; o: " << CGAL_point(pos_zero);
    oss << "; v" << infinite_speed;
    if (infinite_speed != InfiniteSpeedType::NONE) {
      oss << ": (" << CGAL_vector(velocity);
    }
    oss << "; s: (" << CGAL_point(pos_start) << ") @ " << CGAL::to_double(time_start);
    if (has_stopped_) {
      oss << "; e: (" << CGAL_point(pos_stop_) << ") @ " << CGAL::to_double(time_stop_);
    };
  }
  oss  << ")";
  return oss.str();
}

std::ostream& operator<<(std::ostream& os, const WavefrontVertex::LineIntersectionType t) {
  switch (t) {
    case WavefrontVertex::LineIntersectionType::ONE:  return os << "ONE";
    case WavefrontVertex::LineIntersectionType::ALL:  return os << "ALL";
    case WavefrontVertex::LineIntersectionType::NONE: return os << "NONE";
  }
  CANNOTHAPPEN_MSG << "Fell through switch which should cover all cases.";
  assert(false);
  abort();
}

std::ostream&
operator<<(std::ostream& os, const InfiniteSpeedType &a) {
  switch (a) {
    case InfiniteSpeedType::NONE:
      os << "";
      break;
    case InfiniteSpeedType::OPPOSING:
      os << "^";
      break;
    case InfiniteSpeedType::WEIGHTED:
      os << "~";
      break;
  }
  return os;
}
