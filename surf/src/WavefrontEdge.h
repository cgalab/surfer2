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

#include "CollapseSpec.h"
#include "WavefrontSupportingLine.h"
#include "SkeletonDCEL.h"

using WavefrontEdgeList = FixedVector<WavefrontEdge>;
class WavefrontEdge {
  private:
    static unsigned wavefront_edge_ctr;

    const unsigned id;
    bool is_dead_ = false;        /** stopped propagating */
    WavefrontVertex* vertices[2]; /** The left and right wavefront vertex right
                                   *  now.  This changes over the course of the
                                   *  propagation period.
                                   */
    std::shared_ptr<const WavefrontSupportingLine> supporting_line;
                                  /** The supporting line backing this wavefront vertex.  */
    KineticTriangle* incident_triangle_;
                                  /** The triangle incident right now at this wavefront edge.  */
  public:
    const bool is_initial;        /** Is this wavefront edge one that was
                                    * created initially as part of the input or
                                    * from beveling (true), or is it the result
                                    * of a wavefront edge having been split during the
                                    * propagation period (false).
                                    */
    const bool is_beveling;       /** Is this wavefront edge the result of beveling,
                                    * and thus degenerate at time zero?
                                    */
  private:
    WavefrontVertex* initial_vertices[2];
                                  /** The pointers to the left and right straight
                                    * skeleton arcs (==kinetic wavefront vertex).
                                    * Only for is_initial wavefront edges, so
                                    * we can then find the faces nicely. */

  public:
    SkeletonDCELFace * const skeleton_face;
                                  /** The straight skeleton face that this edge traces out (at least partially.
                                   *  With splitting, multiple edges are needed to trace out a single face.
                                   */

 private:
    mutable EdgeCollapseSpec collapse_spec;
    mutable bool collapse_spec_valid = false;
    #ifndef NDEBUG
    mutable WavefrontVertex* collapse_spec_computed_with_vertices[2];
    #endif

  public:
    friend std::ostream& operator<<(std::ostream& os, const WavefrontEdge& e);
    using EdgePtrPair = std::pair<WavefrontEdge*,WavefrontEdge*>;

  public:
    /* Used when setting up initial wavefront edges for all constraints */
    WavefrontEdge(const Point_2 &u, const Point_2 &v, const NT &weight, KineticTriangle* incident_triangle, SkeletonDCELFace * p_skeleton_face)
      : id(wavefront_edge_ctr++)
      , vertices { NULL, NULL }
      , supporting_line( std::make_shared<const WavefrontSupportingLine>(u, v, weight) )
      , incident_triangle_(incident_triangle)
      , is_initial(true)
      , is_beveling(false)
      , initial_vertices { NULL, NULL }
      , skeleton_face(p_skeleton_face)
      #ifndef NDEBUG
      , collapse_spec_computed_with_vertices { NULL, NULL }
      #endif
      {
      assert(!!skeleton_face ^ is_beveling);
    }

    /* Used when setting up bevels */
    WavefrontEdge(std::shared_ptr<const WavefrontSupportingLine> p_supporting_line, SkeletonDCELFace * p_skeleton_face)
      : id(wavefront_edge_ctr++)
      , vertices { NULL, NULL }
      , supporting_line(p_supporting_line)
      , incident_triangle_(NULL)
      , is_initial(true)
      , is_beveling(true)
      , initial_vertices { NULL, NULL }
      , skeleton_face(p_skeleton_face)
      #ifndef NDEBUG
      , collapse_spec_computed_with_vertices { NULL, NULL }
      #endif
      {
      assert(!!skeleton_face ^ is_beveling);
    }

  private:
    /* Used internally for split(). */
    WavefrontEdge(WavefrontVertex* va,
                  WavefrontVertex* vb,
                  std::shared_ptr<const WavefrontSupportingLine> p_supporting_line,
                  KineticTriangle *incident_triangle,
                  SkeletonDCELFace * p_skeleton_face)
      : id(wavefront_edge_ctr++)
      , vertices {va, vb}
      , supporting_line(p_supporting_line)
      , incident_triangle_(incident_triangle)
      , is_initial(false)
      , is_beveling(false)
      , initial_vertices { NULL, NULL }
      , skeleton_face(p_skeleton_face)
      #ifndef NDEBUG
      , collapse_spec_computed_with_vertices { NULL, NULL }
      #endif
      {
      assert(!!skeleton_face ^ is_beveling);
    }
  public:

    void set_dead() {
      assert(!is_dead_);
      is_dead_ = true;
    };

    std::shared_ptr<const WavefrontSupportingLine> const l() const { return supporting_line; }
    KineticTriangle* incident_triangle() const { return incident_triangle_; }
    bool is_dead() const { return is_dead_; }
    inline WavefrontVertex* vertex(unsigned i) const {
      assert(!is_dead_);
      assert(i <= 1);
      return vertices[i];
    }
    inline WavefrontVertex* initial_vertex(unsigned i) const {
      assert(is_initial);
      assert(i <= 1);
      return initial_vertices[i];
    }

    inline void set_wavefrontedge_vertex(unsigned i, WavefrontVertex *v) {
      assert(!is_dead_);
      assert(i <= 1);
      assert(v);
      vertices[i] = v;
      invalidate_collapse_spec();
    }
    inline void set_initial_vertices() {
      assert(is_initial);
      for (int i=0; i<=1; ++i) {
        assert(vertices[i]);
        assert(!initial_vertices[i]);
        initial_vertices[i] = vertices[i];
      };
    }

    void set_incident_triangle(KineticTriangle* incident_triangle);

  public:
    const CollapseSpec get_collapse(int component, const NT& time_now, int collapsing_edge) const {
      assert_edge_sane(collapsing_edge);
      return CollapseSpec(component, get_edge_collapse(time_now), collapsing_edge);
    }
    const EdgeCollapseSpec& get_edge_collapse(const NT& time_now) const {
      assert(!is_dead_);
      assert(vertices[0]);
      assert(vertices[1]);
      if (!collapse_spec_valid) {
        collapse_spec = compute_collapse(time_now);
        collapse_spec_valid = true;
        #ifndef NDEBUG
        set_to_cur_wf_vertices(collapse_spec_computed_with_vertices);
        #endif
      };
      return get_cached_edge_collapse();
    }

    EdgePtrPair split(WavefrontEdgeList& wavefront_edges);

    bool parallel_endpoints(const NT& time_now) const {
      EdgeCollapseSpec e = get_edge_collapse(time_now);
      switch (e.type()) {
        case EdgeCollapseType::UNDEFINED:
          assert(0);
          return 0;
        case EdgeCollapseType::PAST:
        case EdgeCollapseType::FUTURE:
          return false;
        case EdgeCollapseType::ALWAYS:
        case EdgeCollapseType::NEVER:
          return true;
      }
      assert(0);
      return 0;
    }

  private:
    #ifndef NDEBUG
    void assert_edge_sane(int collapsing_edge) const;
    #else
    void assert_edge_sane(int collapsing_edge) const {};
    #endif

    const EdgeCollapseSpec& get_cached_edge_collapse() const {
      assert(collapse_spec_valid);
      #ifndef NDEBUG
      assert_cur_wf_vertices(collapse_spec_computed_with_vertices);
      #endif
      return collapse_spec;
    }

    void invalidate_collapse_spec() {
      collapse_spec_valid = false;
      #ifndef NDEBUG
      invalidte_cur_wf_vertices(collapse_spec_computed_with_vertices);
      #endif
    }

    #ifndef NDEBUG
    void set_to_cur_wf_vertices(WavefrontVertex* arr[2]) const {
      for (unsigned i=0; i<2; ++i) {
        arr[i] = vertices[i];
      }
    }
    void invalidte_cur_wf_vertices(WavefrontVertex* arr[2]) const {
      for (unsigned i=0; i<2; ++i) {
        arr[i] = NULL;
      }
    }
    void assert_cur_wf_vertices(WavefrontVertex* const arr[2]) const {
      for (unsigned i=0; i<2; ++i) {
        assert(arr[i]);
        assert(arr[i] == vertices[i]);
      }
    }
    #endif

    EdgeCollapseSpec compute_collapse(const NT& time_now) const;
};
