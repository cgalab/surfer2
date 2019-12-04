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
#include "SkeletonDCEL.h"
#include "WavefrontEdge.h"

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
