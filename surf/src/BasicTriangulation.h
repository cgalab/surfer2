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

#include "BasicInput.h"

#include <CGAL/Triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>

struct BasicVertexInfo {
  int original_vertex_idx = -1;
};
struct BasicFaceInfo {
  int component = -1;
  bool queued_potential_components = false; /** true if we have visited a neighbor across a constraint. */
  bool queued_this_component = false;       /** true if we have ever visited a neighbor across a triangulation edge.
                                                So it's either queued for the current of a previous component. */
  bool matches_component(int q) {
    return (q < 0 || component == q);
  }
};

class BasicTriangulation : public
    CGAL::Constrained_Delaunay_triangulation_2<
      Kernel,
      CGAL::Triangulation_data_structure_2<
        CGAL::Triangulation_vertex_base_with_info_2<BasicVertexInfo, Kernel>,
        CGAL::Triangulation_face_base_with_info_2<BasicFaceInfo, Kernel, CGAL::Constrained_triangulation_face_base_2<Kernel> >
      >,
      CGAL::No_intersection_tag> {
  private:
    using Base = Constrained_Delaunay_triangulation_2;
    bool consistent_sidedness;
    int max_component_ = -1;

    /** establish components.
     *
     * we provide the verex handles of the first edge (v0, v1), and
     * we start by tagging the component left of that as 0, and the
     * component right of it (if it is a different one) as 1.
     *
     * All other components are then tagged with increasing numbers.
     *
     * If there are only two components, and every edge has a different
     * component on each of its sides, we set consistent_sidedness to true.
     * Else we set it to false.
     */
    void tag_components(const Vertex_handle& v0, const Vertex_handle& v1);
  public:
    /* Initialize the constrained triangulation.
     */
    void initialize(const BasicInput& input);
    int max_component() const { return max_component_; };
};
