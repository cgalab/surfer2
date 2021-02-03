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
#pragma once

#include "surf.h"
#include "cgaltools.h"
#include "tools.h"

#include "CGAL/Arr_dcel_base.h"
#include <CGAL/Polygon_2.h>

#include <boost/variant.hpp>
#include <unordered_set>

class SkeletonDCELVertexBase : public CGAL::Arr_vertex_base<const Point_3> {
  DEBUG_DECL(
    public:
      SkeletonDCELVertexBase() : id(ctr++) {};
    private:
      static unsigned ctr;
    public:
      const unsigned id;
  )
};
/** A halfedge.  The Segment_3/Ray_3 is undirected. */
class SkeletonDCELHalfedgeBase : public CGAL::Arr_halfedge_base<boost::variant<Segment_3,Ray_3>> {
  friend class SkeletonDCEL;
  private:
    bool is_emanating_input_ = false;
  public:
    /** Is this an input edge that is emmanating a wavefront.
     *
     * True for input edges on the side where we do a wavefront propagation.
     */
    bool is_emanating_input() const { return is_emanating_input_; };
    /** Is this an input edge for which we do a wavefront propagation on at least one side.
     *
     * May only be called when class is specialized by Arr_halfedge */
    bool is_input() const;

  DEBUG_DECL(
    public:
      SkeletonDCELHalfedgeBase() : id(ctr++) {};
    private:
      static unsigned ctr;
    public:
      const unsigned id;
  )
};
class SkeletonDCELFaceBase : public CGAL::Arr_face_base {
  private:
    bool is_beveling_face_;
  public:
    void set_is_beveling_face(bool is_beveling_face) { is_beveling_face_ = is_beveling_face; };
    bool is_beveling_face() const { return is_beveling_face_; };

  DEBUG_DECL(
    public:
      SkeletonDCELFaceBase() : id(ctr++) {};
    private:
      static unsigned ctr;
    public:
      const unsigned id;
  )
};

class SkeletonDCEL : public CGAL::Arr_dcel_base<SkeletonDCELVertexBase, SkeletonDCELHalfedgeBase, SkeletonDCELFaceBase> {
  friend class KineticTriangulation;
  private:
    using Base = CGAL::Arr_dcel_base<SkeletonDCELVertexBase, SkeletonDCELHalfedgeBase, SkeletonDCELFaceBase>;
    using Curve = Halfedge::X_monotone_curve;

    FixedVector<Point_3> points;
    FixedVector<Curve> curves;
    int num_v_skew = -1;
  private:
    unsigned get_expected_num_v() const {
      unsigned num_he = size_of_halfedges();
      assert(num_he % 2 == 0);
      unsigned num_f = size_of_faces();
      return 2 + num_he/2 - num_f;
    }

  private:
    Face* setup_new_input_edge(const WavefrontEdge * const buddy_wavefront);

    /* skew normally is -1, since we do not need a point for the infinite vertex.
     * However, if we don't have an infinite vertex (because we only do an
     * interior SK), then the skew does not hold and needs to be one.
     */
    void set_num_v_skew(int skew) {
      assert(points.size() == 0);
      num_v_skew = skew;
    }
    void set_number_of_points_and_curves() {
      unsigned num_he = size_of_halfedges();
      unsigned num_f = size_of_faces();
      assert_sane();
      points.reserve(get_expected_num_v() + num_v_skew);
      curves.reserve(num_he / 2);
    }

    Point_3 * new_point(Point_3&& p) {
      points.emplace_back(std::forward<Point_3>(p));
      Point_3* pp = &points.back();
      return pp;
    }
    Curve * new_segment(Segment_3&& s) {
      curves.emplace_back(std::forward<Segment_3>(s));
      Curve* p = &curves.back();
      return p;
    }
    Curve * new_ray(Ray_3&& s) {
      curves.emplace_back(std::forward<Ray_3>(s));
      Curve* p = &curves.back();
      return p;
    }

  private:
    enum class SegmentType { INPUT, ARC, OFFSET };
    static void write_ipe_segment(std::ostream& os, const Segment_2& s, SegmentType t);
  public:
    void write_ipe(std::ostream& os, const std::string& offset_spec) const;
    void write_obj(std::ostream& os) const;

  public:
    using OffsetFamily = std::vector<Segment_2>;
  protected:
    CGAL::Sign arc_offset_do_intersect(const NT& offsetting_distance, const Halfedge* const he) const;
    Point_2 arc_offset_get_intersect(const Plane_3& offset_plane, const Halfedge* const he) const;
    std::pair<bool, Segment_2> make_one_offset_segment(const NT& offsetting_distance, const Halfedge* const start, std::unordered_set<const Halfedge*>& visited) const;
  public:
    OffsetFamily make_offset(const NT& offsetting_distance) const;
    static std::vector<NT> parse_offset_spec(const std::string& offset_spec);

  public:
    #ifndef SURF_NDEBUG
    void assert_sane() const {
      DBG_FUNC_BEGIN(DBG_SKEL);
      unsigned num_he = size_of_halfedges();
      unsigned num_v = size_of_vertices();
      unsigned num_f = size_of_faces();
      unsigned num_iso = size_of_isolated_vertices();
      unsigned num_outer_ccbs = size_of_outer_ccbs();
      unsigned num_inner_ccbs = size_of_inner_ccbs();

      DBG(DBG_SKEL) << " num_he        : " << num_he;
      DBG(DBG_SKEL) << " num_v         : " << num_v;
      DBG(DBG_SKEL) << " num_f         : " << num_f;
      DBG(DBG_SKEL) << " num_iso       : " << num_iso;
      DBG(DBG_SKEL) << " num_outer_ccbs: " << num_outer_ccbs;
      DBG(DBG_SKEL) << " num_inner_ccbs: " << num_inner_ccbs;
      DBG(DBG_SKEL) << " points.size() : " << points.size();
      DBG(DBG_SKEL) << " curves.size() : " << curves.size();


      assert(num_he % 2 == 0);
      assert(num_v == 0 || num_v == get_expected_num_v());
      assert(num_iso == 0);
      assert(num_outer_ccbs == num_f);
      assert(num_inner_ccbs == 0);
      assert((num_v == 0 && points.size() == 0) || (num_v > 0 && points.size() == num_v+num_v_skew));
      assert(curves.size() == 0 || curves.size() == num_he/2);
      DBG_FUNC_END(DBG_SKEL);
    }
    #else
    void assert_sane() const {};
    #endif
};
using SkeletonDCELVertex = SkeletonDCEL::Vertex;
using SkeletonDCELHalfedge = SkeletonDCEL::Halfedge;
using SkeletonDCELCbb = SkeletonDCEL::Outer_ccb;
using SkeletonDCELFace = SkeletonDCEL::Face;

inline bool
SkeletonDCELHalfedgeBase::
is_input() const {
  static_assert(std::is_base_of<SkeletonDCELHalfedgeBase, SkeletonDCELHalfedge>::value);
  const SkeletonDCELHalfedge * const he = static_cast<const SkeletonDCELHalfedge*>(this);
  assert(he);
  assert(he->opposite());
  return he->is_emanating_input() || he->opposite()->is_emanating_input();
}

#ifndef SURF_NDEBUG
inline std::ostream& operator<<(std::ostream& os, const SkeletonDCELVertex& i) {
  os << "dV" << i.id;
  if (i.has_null_point()) {
    os << "(-)";
  } else {
    os << CGAL_point(i.point());
  }
  return os;
}
inline std::ostream& operator<<(std::ostream& os, const SkeletonDCELHalfedge& i) {
  os << "dH" << i.id;
  if (i.vertex()) {
    os << "[" << *i.vertex() << "]";
  } else {
    os << "(-)";
  }
  return os;
}
inline std::ostream& operator<<(std::ostream& os, const SkeletonDCELFace& i) {
  os << "dF" << i.id;
  return os;
}
#else
inline std::ostream& operator<<(std::ostream& os, const SkeletonDCELVertex& i) { return os << "dV"; }
inline std::ostream& operator<<(std::ostream& os, const SkeletonDCELHalfedge& i) { return os << "dH"; }
inline std::ostream& operator<<(std::ostream& os, const SkeletonDCELFace& i) { return os << "dF"; }
#endif
