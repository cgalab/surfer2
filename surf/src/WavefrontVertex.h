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

#include "WavefrontEdge.h"

enum class InfiniteSpeedType { NONE, OPPOSING, WEIGHTED };
std::ostream& operator<<(std::ostream& os, const InfiniteSpeedType &a);

class WavefrontVertex {
  friend class KineticTriangulationGraphicsItem;
  friend class VertexList;
  private:
    using VertexAngle = CGAL::Sign;
  public:
    static constexpr CGAL::Sign CONVEX = CGAL::LEFT_TURN;
    static constexpr CGAL::Sign STRAIGHT =  CGAL::COLLINEAR;
    static constexpr CGAL::Sign REFLEX = CGAL::RIGHT_TURN;

  private:
    static unsigned kvctr;
    const unsigned id;
  public:
    const Point_2 pos_zero;
    const Point_2 pos_start;
    const NT time_start;
  private:
    const WavefrontEdge* incident_wavefront_edges[2];
    const VertexAngle angle;
  public:
    const bool is_initial;
    const bool is_beveling;
    const bool is_infinite;
    const InfiniteSpeedType infinite_speed; /** This wavefront vertex is
      either between parallel, opposing wavefront elements that have crashed
      into each other and become collinear, or it is between neighboring
      wavefront edges that have become collinear yet have different weights. */
    const Vector_2 velocity;
  private:
    const Polynomial_1 px_, py_;
    bool has_stopped_ = false;
    NT time_stop_;
    Point_2 pos_stop_;

    bool is_degenerate_ = false; /* if pos_stop == pos_start */
    SkeletonDCELHalfedge* skeleton_dcel_halfedge_[2];

    /* wavefront vertices form a doubly-linked edge list to represent
     * the boundary of their left(0) and right(1) incident faces.
     *
     * prev points to the wavefront vertex earlier in time, next to
     * the one later in time, so when traversing a face, care needs
     * to be taken at each arc (i.e. wavefront-vertex) wrt direction.
     */
    WavefrontVertex* next_vertex_[2];
    WavefrontVertex* prev_vertex_[2];

    WavefrontVertex(
      const Point_2& pos_zero,
      const Point_2& pos_start,
      const NT& time_start,
      const WavefrontEdge * const a,
      const WavefrontEdge * const b,
      bool is_initial=false,
      bool is_beveling=false,
      bool is_infinite=false);

  protected:
    /** type of intersections for two lines */
    enum class LineIntersectionType {
      ONE,   /* the two lines intersect in one point */
      ALL,   /* the two lines are parallel and coincide */
      NONE,  /* the two liens are parallel and distinct */
      };
    friend std::ostream& operator<<(std::ostream& os, const WavefrontVertex::LineIntersectionType t);

    static InfiniteSpeedType get_infinite_speed_type(const WavefrontEdge * const a, const WavefrontEdge * const b, const VertexAngle& angle);
    static std::tuple<LineIntersectionType, Point_2> compute_intersection(const Line_2& a, const Line_2& b);

    static Vector_2 compute_velocity(
      const Point_2& pos_zero,
      const WavefrontSupportingLine& a,
      const WavefrontSupportingLine& b,
      const VertexAngle angle);

  public:
    bool is_reflex_or_straight() const { return angle != CONVEX; }
    bool is_convex_or_straight() const { return angle != REFLEX; }
    //bool is_straight() const { return angle == STRAIGHT; }
    bool has_stopped() const { return has_stopped_; }
    const NT& time_stop() const { return time_stop_; }
    const Point_2& pos_stop() const { return pos_stop_; }

    //KineticTriangle const * const * triangles() const { return incident_triangles; };
    WavefrontEdge const * const * wavefronts() const { return incident_wavefront_edges; };

    Point_2 p_at(const NT& t) const {
      assert(!has_stopped_ || t <= time_stop_);
      assert(!is_infinite);
      return pos_zero + velocity*t;
    };

    /** return the position of this vertex for drawing purposes.
     *
     * If the time to draw is later than the stop position, return
     * the stop position.
     *
     * If the time to draw is prior to the start position, no such
     * special handling is done and we return the location where
     * the vertex would have been such that it is at the start position
     * at the start time given its velocity.
     */
    Point_2 p_at_draw(const NT& t) const {
      assert(!is_infinite);
      bool return_stop_pos;

      if (has_stopped_) {
        if (t < time_stop_) {
          return_stop_pos = false;
        } else {
          return_stop_pos = true;
        };
      } else {
        return_stop_pos = false;
      }

      return return_stop_pos ? pos_stop_
                             : (pos_zero + velocity*t);
    };

    void stop(const NT& t) {
      assert(!has_stopped_);
      assert(infinite_speed == InfiniteSpeedType::NONE);
      time_stop_ = t;
      pos_stop_ = p_at(t);
      has_stopped_ = true;

      if (pos_stop_ == pos_start) {
        is_degenerate_ = true;
        assert(time_stop_ == time_start);
      };
    }

    void stop(const NT& t, const Point_2& p) {
      assert(!has_stopped_);
      assert(infinite_speed != InfiniteSpeedType::NONE);
      time_stop_ = t;
      pos_stop_ = p;
      has_stopped_ = true;

      assert(time_stop_ == time_start);

      if (pos_stop_ == pos_start) {
        is_degenerate_ = true;
      };
    }

    const WavefrontEdge* incident_wavefront_edge(unsigned i) const {
      assert(i <= 1);
      return incident_wavefront_edges[i];
    };
    void set_incident_wavefront_edge(unsigned i, const WavefrontEdge* e) {
      assert(i <= 1);
      assert(incident_wavefront_edges[i]);
      assert(e);
      assert(incident_wavefront_edges[i]->l() == e->l());
      incident_wavefront_edges[i] = e;
    };

    WavefrontVertex* next_vertex(unsigned side) const {
      assert(side <= 1);
      return next_vertex_[side];
    }
    WavefrontVertex* prev_vertex(unsigned side) const {
      assert(side <= 1);
      return prev_vertex_[side];
    }

    static WavefrontVertex make_initial_vertex(
      const Point_2& pos_zero,
      const WavefrontEdge* const a,
      const WavefrontEdge* const b,
      bool is_beveling
    ) {
      assert(a->l()->l.has_on(pos_zero));
      assert(b->l()->l.has_on(pos_zero));
      return WavefrontVertex(pos_zero, pos_zero, CORE_ZERO, a, b, true, is_beveling);
    }
    static WavefrontVertex make_infinite_vertex() {
      return WavefrontVertex(CGAL::ORIGIN, CGAL::ORIGIN, CORE_ZERO, NULL, NULL, true, false, true);
    }

  private:
    static WavefrontVertex make_vertex(
      const Point_2& pos,
      const NT& time,
      const WavefrontEdge* const a,
      const WavefrontEdge* const b,
      bool from_split
    );

  public:
    friend inline std::ostream& operator<<(std::ostream& os, const WavefrontVertex * const kv) {
      if (kv) {
        os << "kv"
           << kv->id
           << (kv->angle == CONVEX ? "c" :
               kv->angle == REFLEX ? "r" :
               kv->angle == STRAIGHT ? "=" :
                                       "XXX-INVALID-ANGLE")
           << kv->infinite_speed
           << (kv->has_stopped_ ? "s" : "");
      } else {
        os << "kv*";
      }
      return os;
    }
    std::string details() const;

    const Polynomial_1& px() const {
      assert(!is_infinite);
      assert(!has_stopped_);
      return px_;
    };
    const Polynomial_1& py() const {
      assert(!is_infinite);
      assert(!has_stopped_);
      return py_;
    };

    #ifndef NDEBUG
    void assert_valid() const {
      assert(is_initial || !is_beveling); // !initial => !beveling   <=>  !!initial v !beveling
      for (int i=0; i<2; ++i) {
        if (is_initial) {
          assert(!prev_vertex_[i] || is_beveling);
        } else {
          assert(prev_vertex_[i]);
        }
        assert(! has_stopped() ^ !!next_vertex_[i]);
      }
    };
    #else
    void assert_valid() const {};
    #endif

    // ==================== functions maintaining the DCEL =====================

    /** set the successor in the DCEL
     *
     * Also update their prev (or next) pointer depending on whether we have
     * head_to_tail set to true or not.
     */
    void set_next_vertex(unsigned side, WavefrontVertex* next, bool head_to_tail = true) {
      assert(side <= 1);
      assert(next);
      assert(has_stopped());
      DBG(DBG_KT) << " for " << this << " next_vertex_[" << side << "] is " << next_vertex_[side];
      DBG(DBG_KT) << " for " << this << " next_vertex_[" << side << "] := " << next;
      assert(next_vertex_[side] == NULL);
      next_vertex_[side] = next;

      if (head_to_tail) {
        DBG(DBG_KT) << " +for " << next << " prev_vertex_[" << side << "] is " << next->prev_vertex_[side];
        DBG(DBG_KT) << " +for " << next << " prev_vertex_[" << side << "] := " << this;
        assert(next->prev_vertex_[side] == NULL);
        next->prev_vertex_[side] = this;
      } else {
        /* head to head */
        DBG(DBG_KT) << " +for " << next << " next_vertex_[" << 1-side << "] is " << next->next_vertex_[1-side];
        DBG(DBG_KT) << " +for " << next << " next_vertex_[" << 1-side << "] := " << this;
        assert(next->next_vertex_[1-side] == NULL);
        next->next_vertex_[1-side] = this;
      };
    }

    /** join two wavefront vertices, tail-to-tail.
     *
     * This is used after a split event.  Is called at a, where a's left (ccw)
     * side is towards the split edge, i.e. where prev_vertex[0] is null.
     *
     * This is also used to link initial vertices while beveling.  Going
     * clockwise about a vertex, this is called at each kinetc vertex with
     * the previous one as an argument.
     */
    void link_tail_to_tail(WavefrontVertex* other) {
      assert(prev_vertex_[0] == NULL);
      assert(other->prev_vertex_[1] == NULL);
      prev_vertex_[0] = other;
      other->prev_vertex_[1] = this;
      DBG(DBG_KT) << " For " << this << " prev_vertex_[0] := " << other;
      DBG(DBG_KT) << " For " << other << " prev_vertex_[1] := " << this;
    }

    bool is_degenerate() const { return is_degenerate_; }

    SkeletonDCELHalfedge* skeleton_dcel_halfedge(unsigned i) const {
      assert(i <= 1);

      assert(!is_degenerate());
      return skeleton_dcel_halfedge_[i];
    };

    void set_skeleton_dcel_halfedge(unsigned i, SkeletonDCELHalfedge* he) {
      assert(i <= 1);

      assert(!is_degenerate());
      skeleton_dcel_halfedge_[i] = he;
    }
};

class VertexList : public FixedVector<WavefrontVertex> {
  public:
    WavefrontVertex* make_initial_vertex(
      const Point_2& pos_zero,
      const WavefrontEdge* const a,
      const WavefrontEdge* const b,
      bool is_beveling = false
    ) {
      emplace_back(WavefrontVertex::make_initial_vertex(pos_zero, a, b, is_beveling));
      WavefrontVertex* v = &back();
      return v;
    }

    /** create a new kinetic vertex during the propagation period.
     *  link the vertex to its ancestors on the wavefront sides
     *  that are still propagating.
     *
     *  Linking the predecessors on the no-longer-propagating side
     *  is the responsibility of the caller, as it's not clear
     *  how they should be linked.  (together in the case of
     *  a constraint collapse, but not necessarily in spoke collapses.)
     */
    WavefrontVertex* make_vertex(
      const Point_2& pos,
      const NT& time,
      const WavefrontEdge* const a,
      const WavefrontEdge* const b,
      bool from_split=false
    ) {
      emplace_back(WavefrontVertex::make_vertex(pos, time, a, b, from_split));
      WavefrontVertex* v = &back();
      return v;
    }
};

std::ostream& operator<<(std::ostream& os, const WavefrontVertex::LineIntersectionType t);
