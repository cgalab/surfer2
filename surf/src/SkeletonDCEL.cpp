/**  surfer2 -- a straight skeleton implementation
 *
 *  Copyright 2015 -- 2019 Peter Palfraader
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
#include "SkeletonDCEL.h"
#include "WavefrontEdge.h"


#ifndef NDEBUG
unsigned SkeletonDCELVertexBase::ctr = 0;
unsigned SkeletonDCELHalfedgeBase::ctr = 0;
unsigned SkeletonDCELFaceBase::ctr = 0;
#endif

#include <unordered_set>

SkeletonDCELFace*
SkeletonDCEL::
setup_new_input_edge(const WavefrontEdge * const buddy_wavefront) {
  SkeletonDCELFace * face = new_face();
  SkeletonDCELCbb * ccb = new_outer_ccb();
  SkeletonDCELHalfedge * halfedge = NULL;

  ccb->set_face(face);

  if (buddy_wavefront) { /* buddy wavefront already set up */
    halfedge = (*buddy_wavefront->skeleton_face->outer_ccbs_begin())->opposite();
    assert(halfedge->is_on_outer_ccb());
    assert(halfedge->outer_ccb() == NULL);
  } else {
    halfedge = new_edge();
  }
  halfedge->is_input_ = true;
  halfedge->set_outer_ccb(ccb);

  face->add_outer_ccb(ccb, halfedge);
  return face;
}

void
SkeletonDCEL::
write_obj(std::ostream& os) const {
  DBG(DBG_SKEL) << " dumping points";
  for (const Point_3& p : points) {
    DBG(DBG_SKEL) << "  dumping point";
    os << "v " << CGAL::to_double(p.x())
       << " " << CGAL::to_double(p.y())
       << " " << CGAL::to_double(p.z()) << std::endl;
  };
  os << std::endl;
  DBG(DBG_SKEL) << " dumping faces";
  for (const SkeletonDCELFace& f : faces) {
    const SkeletonDCELHalfedge* first = * f.outer_ccbs_begin();
    const SkeletonDCELHalfedge* he = first;
    os << "f";
    const Point_3* start_of_points = &*points.begin();
    do {
      const SkeletonDCELVertex* v = he->vertex();
      assert(v);
      if (v->has_null_point()) {
        /* infinity */
      } else {
        const Point_3& p = v->point();
        os << " " << (1 + (&p - start_of_points));
      }

      he = he->next();
    } while (he != first);
    os << std::endl;
  };
}

SkeletonDCEL::OffsetCurve
SkeletonDCEL::make_one_offset_curve(const Plane_3& offset_plane, const SkeletonDCELHalfedge* const start, std::unordered_set<const SkeletonDCELHalfedge*>& visited) const {
  DBG_FUNC_BEGIN(DBG_SKEL);
  DBG(DBG_SKEL) << "Making offset curve.";
  OffsetCurve offset;
  const SkeletonDCELHalfedge* he = start;
  do {
    auto [ it, inserted ] = visited.insert(he);
    assert(inserted);

    const Curve& arc = he->curve();
    boost::optional< boost::variant<Point_3, Segment_3, Ray_3> > intersect;

    if (arc.type() == typeid(Segment_3)) {
      const Segment_3& s = boost::get<Segment_3>(arc);
      intersect = intersection(s, offset_plane);
    } else {
      assert(arc.type() == typeid(Ray_3));
      const Ray_3& r = boost::get<Ray_3>(arc);
      intersect = intersection(r, offset_plane);
    }

    if (!intersect) {
      he = he->prev();
    } else {
      if (const Point_3* p = boost::get<Point_3>(&*intersect)) {
        /* XXX Check if p is a vertex of he. */

        offset.push_back( Point_2(p->x(), p->y()) );
        DBG(DBG_SKEL) << "Adding vertex to offset curve: " << CGAL_point(*p);

        auto [ _, inserted2 ] = visited.insert(he->opposite());
        assert(inserted2);
        he = he->opposite()->prev();
      } else if (const Segment_3* s = boost::get<Segment_3>(&*intersect)) {
        NOTIMPL_MSG << "Offset intersection with arc is a segment.";
        assert(false);
        exit(1);
      } else {
        LOG(ERROR) << "Unexpected result of intersection at " << __FILE__ << ":" << __LINE__;
        abort();
      }
    }
  } while (he != start);
  DBG_FUNC_END(DBG_SKEL);
  return offset;
}

SkeletonDCEL::OffsetFamily
SkeletonDCEL::
make_offset(const NT& offsetting_distance) const {
  DBG_FUNC_BEGIN(DBG_SKEL);
  DBG(DBG_SKEL) << "making offset at " << CGAL::to_double(offsetting_distance);

  OffsetFamily offsets;

  std::unordered_set<const SkeletonDCELHalfedge*> visited;
  Plane_3 offset_plane(CORE_ZERO, CORE_ZERO, CORE_ONE, -offsetting_distance);

  for (const SkeletonDCELHalfedge& he : halfedges) {
    if (visited.find(&he) != visited.end()) {
      DBG(DBG_SKEL) << "Not considering " << he << " as already visited";
      continue;
    };
    DBG(DBG_SKEL) << "Considering " << he;

    const SkeletonDCELVertex* const t = he.opposite()->vertex(); // tail
    const SkeletonDCELVertex* const h = he.vertex();             // head

    const SkeletonDCELHalfedge* he_start = NULL;
    if (t->has_null_point() || h->has_null_point()) { // ray
      assert(!t->has_null_point() || !h->has_null_point());

      bool he_is_outgoing = h->has_null_point();
      const NT z = he_is_outgoing ? t->point().z() : h->point().z();
      if (z <= offsetting_distance) {
        he_start = he_is_outgoing ? &he : he.opposite();
      }
    } else {
      const NT zt = t->point().z();
      const NT zh = h->point().z();

      auto [z0, z1] = sorted_tuple(zt, zh);
      if (z0 <= offsetting_distance && offsetting_distance <= z1) {
        auto sign = (zh-zt).sign();
        if (sign == CGAL::POSITIVE) { /* he is pointing along the propagation direction */
          he_start = &he;
        } else if (sign == CGAL::NEGATIVE) { /* he is pointing against the propagation direction */
          he_start = he.opposite();
        } else { /* he is an arc that is horizontal in time. */
          he_start = &he;
        }
      }
    }

    if (he_start) {
      DBG(DBG_SKEL) << "Constructing an offset starting at ray " << he << "; " << *he.prev()->vertex() << "->" << *he.vertex();
      OffsetCurve offset = make_one_offset_curve(offset_plane, he_start, visited);
      if (offset.size() >= 2) {
        offsets.emplace_back( std::move(offset) );
      } else {
        DBG(DBG_SKEL) << "Dropping offset curve with only " << offset.size() << " element(s)";
      }
    } else {
      DBG(DBG_SKEL) << "Not using " << he << " as it does not intersect offset plane";
    }
  };

  // assert( std::all_of(halfedges.begin(), halfedges.end(), [visited](const SkeletonDCELHalfedge& he){return visited.find(&he) != visited.end();}) );
  DBG_FUNC_END(DBG_SKEL);
  return offsets;
}

std::vector<NT>
SkeletonDCEL::
parse_offset_spec(const std::string& offset_spec) {
  DBG_FUNC_BEGIN(DBG_SKEL);
  char *spc = strdup(offset_spec.c_str());
  char *saveptr_block = NULL;
  char *saveptr_offset = NULL;
  char *one_block, *one_offset;
  NT time;
  NT step;
  std::vector<NT> list;

  for (one_block = strtok_r(spc, ",", &saveptr_block);
       one_block != NULL;
       one_block = strtok_r(NULL, ",", &saveptr_block)) {
    DBG(DBG_SKEL) << " Got " << one_block;
    time = 0.;
    for (one_offset = strtok_r(one_block, "+", &saveptr_offset);
         one_offset != NULL;
         one_offset = strtok_r(NULL, "+", &saveptr_offset)) {
      int i, cnt = 1;
      char *m = strchr(one_offset,'*');
      DBG(DBG_SKEL) << "  Got " << one_offset;
      if (m) {
        *m = '\0';
        cnt = atoi(one_offset);
        one_offset = m+1;
      }

      while (*one_offset && *one_offset != '.' && (*one_offset < '0' || *one_offset > '9')) {
        ++one_offset;
      }
      char *e = one_offset;
      while (*e && (*e == '.' || (*e >= '0' && *e <= '9'))) {
        ++e;
      }
      if (*e) *e = '\0';

      step = NT(one_offset);
      DBG(DBG_SKEL) << "    adding " << cnt << " times " << one_offset << "("<<CGAL::to_double(step)<<")";
      for (i=0; i<cnt; i++) {
        time +=  step;
        DBG(DBG_SKEL) << "      adding at time " << CGAL::to_double(time);
        list.emplace_back(time);
      }
    }
  }

  free(spc);
  DBG_FUNC_END(DBG_SKEL);
  return list;
}
