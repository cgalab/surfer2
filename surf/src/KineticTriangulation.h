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

#include "BasicTriangulation.h"
#include "TriangulationUtils.h"
#include "KineticTriangle.h"
#include "WavefrontEdge.h"
#include "WavefrontVertex.h"

#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <vector>
#include <utility>
#include <deque>

class KineticTriangulation {
  private:
    using FaceToTriangleIdxMap = std::unordered_map<BasicTriangulation::Face_handle, const unsigned>;
    using TriangleOriginalVertexIndexList = FixedVector<int>;

    static inline unsigned cw  (int i) { return TriangulationUtils::cw  (i); }
    static inline unsigned ccw (int i) { return TriangulationUtils::ccw (i); }
    static inline unsigned mod3(int i) { return TriangulationUtils::mod3(i); }

  private:
    /* Iterate around the vertex identified by v_in_t_idx in t,
     * either clockwise (++) or counterclockwise (--).
     *
     * The neighborhood relation must be consistent across boundaries we traverse.
     */
    class AroundVertexIterator //:
      //public std::iterator<std::bidirectional_iterator_tag
      //public std::iterator<std::forward_iterator_tag,
      //                     std::pair<KineticTriangle *,unsigned>>
    {
      public:
        using iterator_category = std::bidirectional_iterator_tag;
        //using value_type = std::pair<KineticTriangle *,unsigned>;
        using value_type = void;
        using difference_type = void;
        using pointer = void;
        using reference = void;
      private:
        KineticTriangle* t_;
        unsigned v_in_t_idx_;

        KineticTriangle* next_triangle(const unsigned direction[3]) const {
          assert(v_in_t_idx_ < 3);
          return t_->neighbor(direction[v_in_t_idx_]);
        }

        AroundVertexIterator& walk_dir(const unsigned direction[3]) {
          assert(v_in_t_idx_ < 3);
          KineticTriangle* next = next_triangle(direction);
          unsigned v_in_n_idx;
          if (next) {
            unsigned t_in_next_idx = next->index(t_);
            assert(t_in_next_idx < 3);
            v_in_n_idx = direction[t_in_next_idx];
          } else {
            v_in_n_idx = 0;
          }
          t_ = next;
          v_in_t_idx_ = v_in_n_idx;
          return *this;
        }

        AroundVertexIterator most_dir(const unsigned direction[3]) const {
          AroundVertexIterator res = *this;
          while (res.next_triangle(direction)) {
            res.walk_dir(direction);
            assert(res != *this);
          }
          return res;
        }
      public:
        AroundVertexIterator(KineticTriangle *t=NULL, unsigned v_in_t_idx=0)
          : t_(t)
          , v_in_t_idx_(v_in_t_idx)
        {
          assert(v_in_t_idx < 3);
        }

        const KineticTriangle* next_triangle_cw () const { return next_triangle(TriangulationUtils::_cw ); }
        const KineticTriangle* next_triangle_ccw() const { return next_triangle(TriangulationUtils::_ccw); }
        AroundVertexIterator most_cw () const { return most_dir(TriangulationUtils::_cw ); }
        AroundVertexIterator most_ccw() const { return most_dir(TriangulationUtils::_ccw); }

        AroundVertexIterator& operator++() { return walk_dir(TriangulationUtils::_cw); }
        AroundVertexIterator& operator--() { return walk_dir(TriangulationUtils::_ccw); }
        bool operator==(AroundVertexIterator const & other) const {
          return this->t_ == other.t_ && this->v_in_t_idx_ == other.v_in_t_idx_;
        }
        bool operator!=(AroundVertexIterator const & other) const {
          return !(*this == other);
        }
        KineticTriangle& operator*() { return *t_; }

        KineticTriangle* t() const { return t_; };
        unsigned v_in_t_idx() const { return v_in_t_idx_; }
    };
    friend std::ostream& operator<<(std::ostream& os, const KineticTriangulation::AroundVertexIterator it);
  public:
    using TriangleList = FixedVector<KineticTriangle>;
  private:
    bool initialized = false;
    bool finalized = false;
    WavefrontEdgeList* wavefront_edges = NULL;
    VertexList vertices;
    TriangleList triangles;
    int restrict_component_ = -1;
  public:
    int restrict_component() const { return restrict_component_; };
  private:
    /** The straight skeleton as a DCEL data structure.
     *
     * This is built in two parts.  Initially, we only create halfedges for all
     * input segments, and corresponding faces.  We do not created faces or
     * anything for beveled vertices.
     *
     * Once the propagation is done, we walk through all wavefront vertices (which
     * correspond to arcs in the straight skeleton and created the corresponding
     * halfedges and, where requred (beveling), also new faces.
     * Here we also create vertices.
     */
    SkeletonDCEL skeleton;

    std::shared_ptr<EventQueue> queue;

    std::tuple<Point_2, bool>
    get_vertex_pos(const BasicInput& input,
      const TriangleOriginalVertexIndexList& triangle_original_vertex_indices,
      const KineticTriangle * t,
      unsigned i);

    KineticTriangle * split_vertex(
      KineticTriangle *t,
      unsigned i,
      WavefrontEdge * new_e,
      WavefrontVertex * new_v);
    void create_bevels_at_vertex(
      const BasicInput& input,
      const BasicTriangulation& ct,
      const TriangleOriginalVertexIndexList& triangle_original_vertex_indices,
      KineticTriangle *t,
      unsigned i);

    static unsigned get_basic_vertex_idx_from_triangle_vertex_indices(
      const BasicInput& input,
      const TriangleOriginalVertexIndexList& triangle_original_vertex_indices,
      unsigned t_idx,
      unsigned i);
    static const BasicVertex& get_basic_vertex_from_triangle_vertex_indices(
      const BasicInput& input,
      const TriangleOriginalVertexIndexList& triangle_original_vertex_indices,
      unsigned t_idx,
      unsigned i);
    static void invalidate_basic_vertex_idx_in_triangle_vertex_indices(
      const BasicInput& input,
      TriangleOriginalVertexIndexList& triangle_original_vertex_indices,
      unsigned t_idx,
      unsigned i);

    unsigned get_num_initial_triangles(const BasicTriangulation& ct);
    void initialize_tds(
      const BasicInput& input,
      const BasicTriangulation& ct,
      const unsigned num_initial_triangles,
      FaceToTriangleIdxMap& face_to_triangle_idx,
      TriangleOriginalVertexIndexList& triangle_original_vertex_indices
      );
    void create_supporting_lines(
      const BasicInput& input,
      const BasicTriangulation& ct,
      const unsigned num_initial_triangles,
      const FaceToTriangleIdxMap& face_to_triangle_idx,
      const TriangleOriginalVertexIndexList& triangle_original_vertex_indices);
    void create_kinetic_vertices(
      const BasicInput& input,
      const BasicTriangulation& ct,
      TriangleOriginalVertexIndexList& triangle_original_vertex_indices);
    void create_bevels(
      const BasicInput& input,
      const BasicTriangulation& ct,
      const TriangleOriginalVertexIndexList& triangle_original_vertex_indices);
    void store_initial_wavefront_vertices();
  public:
    void create_remaining_skeleton_dcel();
  private:
    void link_dcel_halfedges_on_ignored_side();
    void set_dcel_vertex(SkeletonDCELHalfedge* start, const Point_2* p, const NT& time);
    SkeletonDCELHalfedge* create_remaining_skeleton_dcel_one_face(WavefrontVertex* start, SkeletonDCELHalfedge* base, SkeletonDCELCbb* c, SkeletonDCELFace* f);

    AroundVertexIterator incident_faces_iterator(KineticTriangle *t, unsigned v_in_t) {
      return AroundVertexIterator(t, v_in_t);
    }
    AroundVertexIterator incident_faces_end() const { return AroundVertexIterator(); }

  public:
    KineticTriangulation() {};
    void initialize(
      const BasicInput& input,
      WavefrontEdgeList* wavefront_edges,
      int p_restrict_component = -1
      );
    void assert_valid() const;
    void assert_valid(int current_component, const NT& time) const;

    void set_queue(std::shared_ptr<EventQueue> q) { queue = q; }

    //const FixedVector<WavefrontVertex>& get_vertices() const { return vertices; };
    const VertexList::const_iterator vertices_begin() const { return vertices.begin(); }
    const VertexList::const_iterator vertices_end() const { return vertices.end(); }

    const TriangleList::const_iterator triangles_begin() const { return triangles.begin(); }
    const TriangleList::const_iterator triangles_end() const { return triangles.end(); }
    unsigned triangles_size() const { return triangles.size(); }

    unsigned event_type_counter[int(CollapseType::NEVER)] = {0};

  private:
    void do_raw_flip(KineticTriangle* t, unsigned edge_idx, const NT& time, bool allow_collinear);
    void do_flip(KineticTriangle* t, unsigned edge_idx, const NT& time, bool allow_collinear=false);
    void do_flip_event(const NT& time, KineticTriangle& t, int edge_idx);

    void do_spoke_collapse_part2(KineticTriangle& t, unsigned edge_idx, const NT& time);
    void do_constraint_collapse_part2(KineticTriangle& t, unsigned edge_idx, const NT& time);

    void handle_constraint_event(const Event& event);
    void handle_face_with_infintely_fast_vertex(const Event& event);
    void handle_spoke_collapse_event(const Event& event);
    void handle_triangle_collapse_event(const Event& event);
    void handle_split_event(const Event& event);
    void handle_split_or_flip_refine_event(const Event& event);
    void handle_vertex_moves_over_spoke_event(const Event& event);
    void handle_ccw_vertex_leaves_ch_event(const Event& event);


    /** Transfer constraints to neighbors
     *
     * Face f is going away because it collapsed.  Transfer the constraints
     * where appropriate.
     *
     * f is a triangle with edges a, b, c.  edge a has collapsed.  Not both
     * of b and c may be constrained.
     *
     * If either b or c is constrained, move the constraint to the neighbor
     * on the other side.
     */
    static void move_constraints_to_neighbor(KineticTriangle& t, int idx);

    /** Note that triangle t has been modified.
     *
     * Put it in out check_refinement queue (at front or back), and
     * put it in the needs_update queue of the event queue.
     */
    void modified(KineticTriangle* t, bool front=false);

    ///////////////
    // Refinement things
    std::deque<KineticTriangle *> check_refinement;
    FixedVector<bool> tidx_in_check_refinement;

    /** refines the triangulation locally around t
     *
     * refines the triangulation locally around t with at most one flip.
     */
    void refine_triangulation(KineticTriangle* t, const NT& time);
    void refine_triangulation_initial();

    void process_check_refinement_queue(const NT& time);
    void put_on_check_refinement(KineticTriangle* t, bool front=false);
    KineticTriangle* check_refinement_pop();
    ///////////////

  public:
    void handle_event(const Event& event);
    const SkeletonDCEL& get_skeleton() const { return skeleton; };
};
