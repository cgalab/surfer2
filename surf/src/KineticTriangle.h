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

#include "TriangulationUtils.h"
#include "WavefrontVertex.h"
#include "CollapseSpec.h"
#include <stddef.h>

/* A triangle, part of our kinetic triangulation.
 *
 * We maintain a kinetic triangulation of the area not yet swept by the
 * wavefront.  When sweeping to the outside, this area can be unbounded.
 *
 * - A triangle of this triangulation always has three vertices (one of which
 *   may be the "infinite" vertex.
 * - Triangle edges may be part of the wavefront, in which case we point to the
 *   input edge from which the corresponding wavefront edge emanated.
 * - Also, each triangle has up to three neighbors, exactly one for each
 *   non-constrained edge.  Thus, each side either has a neighbor or a wavefront set.
 *
 * As such, the kinetic triangulation is a triangulation of a set of polygons,
 * one of them unbounded.
 */
class KineticTriangle {
  friend class KineticTriangulation;

  private:
    static unsigned ktctr;

  public:
    const unsigned id;
    const int component;

  private:
    bool is_dead_ = false;
    bool is_dying_ = false;
    WavefrontVertex* vertices[3];
    WavefrontEdge* wavefronts[3];
    KineticTriangle* neighbors[3];

  private:
    mutable CollapseSpec collapse_spec;
    mutable bool collapse_spec_valid = false;
    #ifndef NDEBUG
    mutable WavefrontVertex* collapse_spec_computed_with_vertices[3];
    #endif

  public:
    static inline int cw (int i) { return TriangulationUtils::cw (i); }
    static inline int ccw(int i) { return TriangulationUtils::ccw(i); }

  public:
    KineticTriangle(int p_component)
      : id(ktctr++)
      , component(p_component)
      , vertices()
      , wavefronts { NULL, NULL, NULL }
      , neighbors()
      , collapse_spec(p_component)
      #ifndef NDEBUG
      , collapse_spec_computed_with_vertices { NULL, NULL, NULL }
      #endif
      {
    }
    KineticTriangle(const KineticTriangle&) = delete;
    KineticTriangle& operator =(const KineticTriangle&) = delete;
    KineticTriangle(KineticTriangle&&) = default;
    KineticTriangle& operator = (KineticTriangle&&) = default;

    inline void set_neighbors(KineticTriangle *n0, KineticTriangle *n1, KineticTriangle *n2);
    inline void set_wavefronts(WavefrontEdge *w0, WavefrontEdge *w1, WavefrontEdge *w2);
    /** Set a vertex of the triangle, updating the wavefront edge if this side is constrained. */
    void set_vertex(unsigned i, WavefrontVertex *v);
    inline bool has_neighbor(KineticTriangle const * const needle) const;
    inline bool has_vertex(WavefrontVertex const * const needle) const;
    inline bool has_wavefront(WavefrontEdge const * const needle) const;
    inline unsigned index(KineticTriangle const * const needle) const;
    inline unsigned index(WavefrontVertex const * const needle) const;
    inline unsigned index(WavefrontEdge const * const needle) const;
    inline KineticTriangle* neighbor(unsigned i) const;
    /** Check if this side has a constraint.
     *
     * This usually implies it has no neighbor, except during construction and while
     * manipulating the triangulation.  Outside of methods, it should always hold
     * that we have exactly either neighbor[i] or constraint[i].
     */
    inline bool is_constrained(unsigned i) const;
    inline WavefrontEdge* wavefront(unsigned i) const;
    const WavefrontVertex * vertex(unsigned i) const { assert(i<3); return vertices[i]; };

    #ifndef NDEBUG
    void assert_valid() const;
    #else
    void assert_valid() const {};
    #endif

    friend std::ostream& operator<<(std::ostream& os, const KineticTriangle * const kt);
    std::string get_name() const {
      return "kt" + std::to_string(id);
    }

    inline void assert_is_id(unsigned q) const { assert(id == q); };

  public:
    const CollapseSpec& get_collapse(const NT& time_now) const {
      assert(!is_dead_);
      assert(!is_dying_);
      if (!collapse_spec_valid) {
        collapse_spec = compute_collapse(time_now);
        collapse_spec_valid = true;
        #ifndef NDEBUG
        set_to_cur_wf_vertices(collapse_spec_computed_with_vertices);
        #endif
      };
      return get_cached_collapse();
    }

  private:
    const CollapseSpec& refine_collapse_spec(CollapseSpec&& c) {
      assert(collapse_spec_valid);
      #ifndef NDEBUG
      assert_cur_wf_vertices(collapse_spec_computed_with_vertices);
      #endif

      assert(collapse_spec.allows_refinement_to(c));

      collapse_spec = std::forward<CollapseSpec>(c);
      return collapse_spec;
    }

    const CollapseSpec& get_cached_collapse() const {
      assert(!is_dying_);
      assert(collapse_spec_valid);
      #ifndef NDEBUG
      assert_cur_wf_vertices(collapse_spec_computed_with_vertices);
      #endif
      return collapse_spec;
    }

    void invalidate_collapse_spec() {
      assert(!is_dead_);
      collapse_spec_valid = false;
      #ifndef NDEBUG
      invalidate_cur_wf_vertices(collapse_spec_computed_with_vertices);
      #endif
      /*
      _is_squeezed = false;
      determinant_valid = false;
      invalidate_cur_wf_vertices(determinantComputedWithWavefrontVertices);
      */
    }

    #ifndef NDEBUG
    inline void set_to_cur_wf_vertices(WavefrontVertex* arr[3]) const;
    inline void invalidate_cur_wf_vertices(WavefrontVertex* arr[3]) const;
    inline void assert_cur_wf_vertices(WavefrontVertex* const arr[3]) const;
    #endif

    CollapseSpec compute_collapse(const NT& time_now) const;
    CollapseSpec compute_collapse_bounded(const NT& time_now) const;
    CollapseSpec compute_collapse_unbounded(const NT& time_now) const;

    CollapseSpec compute_collapse_bounded_constrained_0(const NT& time_now) const;
    CollapseSpec compute_collapse_bounded_constrained_1(const NT& time_now) const;
    CollapseSpec compute_collapse_bounded_constrained_2(const NT& time_now) const;
    CollapseSpec compute_collapse_bounded_constrained_3(const NT& time_now) const;

    static Polynomial_1 compute_determinant_from_vertices(const WavefrontVertex * const v0, const WavefrontVertex * const v1, const WavefrontVertex * const v2);
    static CollapseSpec event_that_will_not_happen(int component, const NT& time_now, const Polynomial_1& determinant);
    static bool get_generic_collapse_time(const NT& time_now, const Polynomial_1& det, NT& collapse_time);
    static bool accept_collapse_bounded_constrained_1(const NT& edge_collapse_time, const Polynomial_1& determinant, bool collapse_time_is_edge_collapse);
    CollapseSpec get_generic_collapse(const NT& time_now, const Polynomial_1& det) const;

    // CollapseSpec compute_constraint_collapse(const NT& time_now, const Polynomial_1& determinant) const;
    // CollapseSpec compute_split_event(const NT& time_now) const;
    // CollapseSpec determine_split_or_flip_bounded_constrained_1(const NT& collapse_time, unsigned c_idx) const;
    CollapseSpec compute_split_or_flip_event_bounded_constrained_1(const NT& time_now, unsigned c_idx, const Polynomial_1& det) const;
    CollapseSpec compute_flip_event(const NT& time_now, const Polynomial_1& determinant) const;

    /* called from KineticTriangulation only */
    void move_constraint_from(unsigned idx, KineticTriangle &src, unsigned src_idx);
    void set_dying() { is_dying_ = true; };
    inline void set_neighbor(unsigned idx, KineticTriangle *n);

    /** set wavefront and update wavefront's incident triangle.
     *
     * - wavefront needs to be not NULL,
     * - wavefront's vertices already need to be right,
     * - there needs to be a neighbor on the side that is about to become a
     *   wavefront, and
     * - that neighbor needs to be dying.
     * - wavefront's (old) incident_triangle needs to be neighbor
     *
     * - sets neighbor to NULL
     * - sets wavefront
     *   sets wavefront's incident_triangle
     */
    void set_wavefront(unsigned idx, WavefrontEdge *e);
    void do_raw_flip_inner(unsigned edge_idx);
    void do_raw_flip(unsigned edge_idx);

  public:
    bool is_dead() const { return is_dead_; }
    /* called by EventQ */
    bool is_dying() const { return is_dying_; }
    /** Mark this triangle as dead.  May only be called once. */
    void set_dead();
    bool is_collapse_spec_valid() const { return collapse_spec_valid; };

    inline bool unbounded() const;
    inline unsigned infinite_vertex_idx() const;
    InfiniteSpeedType has_vertex_infinite_speed() const;
    inline unsigned infinite_speed_opposing_vertex_idx() const;

  private:
    enum class VertexOnSupportingLineType : short { ONCE, NEVER, ALWAYS };
    friend std::ostream& operator<<(std::ostream& os, const KineticTriangle::VertexOnSupportingLineType a);

    static std::tuple<NT, VertexOnSupportingLineType> get_time_vertex_on_supporting_line(const WavefrontVertex& v, const WavefrontSupportingLine& e);
    static CGAL::Sign edge_is_faster_than_vertex(const WavefrontVertex& v, const WavefrontSupportingLine& e);
};
std::ostream& operator<<(std::ostream& os, const KineticTriangle::VertexOnSupportingLineType a);

void
KineticTriangle::
set_neighbors(KineticTriangle *n0, KineticTriangle *n1, KineticTriangle *n2) {
  neighbors[0] = n0;
  neighbors[1] = n1;
  neighbors[2] = n2;
  assert(!n0 || n0->component == component);
  assert(!n1 || n1->component == component);
  assert(!n2 || n2->component == component);
}

void
KineticTriangle::
set_wavefronts(WavefrontEdge *w0, WavefrontEdge *w1, WavefrontEdge *w2) {
  wavefronts[0] = w0;
  wavefronts[1] = w1;
  wavefronts[2] = w2;
  invalidate_collapse_spec();
}
bool
KineticTriangle::
has_neighbor(KineticTriangle const * const needle) const {
  return
    neighbors[0] == needle ||
    neighbors[1] == needle ||
    neighbors[2] == needle;
}

bool
KineticTriangle::
has_vertex(WavefrontVertex const * const needle) const {
  return
    vertices[0] == needle ||
    vertices[1] == needle ||
    vertices[2] == needle;
}

bool
KineticTriangle::
has_wavefront(WavefrontEdge const * const needle) const {
  return
    wavefronts[0] == needle ||
    wavefronts[1] == needle ||
    wavefronts[2] == needle;
}

unsigned
KineticTriangle::
index(KineticTriangle const * const needle) const {
  SRF_precondition(has_neighbor(needle));

  unsigned idx =
    (neighbors[0] == needle) ? 0 :
    (neighbors[1] == needle) ? 1 :
    2;
  return idx;
}

unsigned
KineticTriangle::
index(WavefrontVertex const * const needle) const {
  SRF_precondition(has_vertex(needle));

  unsigned idx =
    (vertices[0] == needle) ? 0 :
    (vertices[1] == needle) ? 1 :
    2;
  return idx;
}

unsigned
KineticTriangle::
index(WavefrontEdge const * const needle) const {
  SRF_precondition(has_wavefront(needle));

  unsigned idx =
    (wavefronts[0] == needle) ? 0 :
    (wavefronts[1] == needle) ? 1 :
    2;
  return idx;
}

KineticTriangle*
KineticTriangle::
neighbor(unsigned i) const {
  SRF_precondition(i < 3);
  return neighbors[i];
}

bool
KineticTriangle::
is_constrained(unsigned i) const {
  SRF_precondition(i < 3);
  return (!!wavefronts[i]);
}

WavefrontEdge*
KineticTriangle::
wavefront(unsigned i) const {
  SRF_precondition(i < 3);
  return wavefronts[i];
}

#ifndef NDEBUG
void
KineticTriangle::
set_to_cur_wf_vertices(WavefrontVertex* arr[3]) const {
  for (unsigned i=0; i<3; ++i) {
    arr[i] = vertices[i];
  }
}

void
KineticTriangle::
invalidate_cur_wf_vertices(WavefrontVertex* arr[3]) const {
  for (unsigned i=0; i<3; ++i) {
    arr[i] = NULL;
  }
}

void
KineticTriangle::
assert_cur_wf_vertices(WavefrontVertex* const arr[3]) const {
  for (unsigned i=0; i<3; ++i) {
    assert(arr[i] == vertices[i]);
  }
}
#endif

void
KineticTriangle::
set_neighbor(unsigned idx, KineticTriangle *n) {
  CGAL_precondition(idx < 3);
  neighbors[idx] = n;
  assert(!n || n->component == component);
}

/** return the index of one vertex with infinite speed.
 */
unsigned
KineticTriangle::
infinite_speed_opposing_vertex_idx() const { // {{{
  //assert(unbounded());
  assert((vertex(0)->infinite_speed == InfiniteSpeedType::OPPOSING) +
         (vertex(1)->infinite_speed == InfiniteSpeedType::OPPOSING) +
         (vertex(2)->infinite_speed == InfiniteSpeedType::OPPOSING) >= 1);
  unsigned idx =
    (vertex(0)->infinite_speed == InfiniteSpeedType::OPPOSING) ? 0 :
    (vertex(1)->infinite_speed == InfiniteSpeedType::OPPOSING) ? 1 :
    2;
  return idx;
} // }}}

unsigned
KineticTriangle::
infinite_vertex_idx() const { // {{{
  //assert(unbounded());
  assert(vertex(0)->is_infinite + vertex(1)->is_infinite + vertex(2)->is_infinite == 1);
  unsigned idx =
    (vertex(0)->is_infinite) ? 0 :
    (vertex(1)->is_infinite) ? 1 :
    2;
  return idx;
} // }}}

bool
KineticTriangle::
unbounded() const { /// {{{
  return(vertex(0)->is_infinite ||
         vertex(1)->is_infinite ||
         vertex(2)->is_infinite);
} // }}}
