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
#include "BasicTriangulation.h"

#include <queue>



void
BasicTriangulation::tag_components(const Vertex_handle& v0, const Vertex_handle& v1) {
  DBG_FUNC_BEGIN(DBG_INPUT);
  DBG(DBG_INPUT) << "establishing components";
  assert(is_edge(v0, v1));

  Face_handle face = NULL;
  /* Find the face incident to edge (v0,v1) */
  {
    auto first_face = incident_faces(v0);  /* This breaks CGAL when there are only infinte faces. */
    assert(!is_infinite(first_face));
    auto current_face = first_face;
    int v0_in_face;
    int v1_in_face;
    do {
      assert(current_face->has_vertex(v0));
      v0_in_face = current_face->index(v0);
      if (current_face->vertex(ccw(v0_in_face)) == v1) {
        face = current_face;
        break;
      }
      ++current_face;
    } while (current_face != first_face);

    assert(face != NULL);
    DEBUG_STMT(v1_in_face = ccw(v0_in_face));

    assert(face->vertex(v0_in_face) == v0);
    assert(face->vertex(v1_in_face) == v1);
    DBG(DBG_EVENTQ) << "found face.";
  };

  std::queue<Face_handle> potential_components;
  face->info().queued_potential_components = true;
  potential_components.push(face);

  consistent_sidedness = true;
  int component_ctr = -1;
  while (!potential_components.empty()) {
    /* o main loop invariant:
     *   for each face f:
     *     f->info().component >= 0  <=>  queued_this_component && queued_potential_components
     *
     *    i.e., once we handle a face, we set both queued_this_component and queued_potential_components.
     *
     * o in the inner loop, we set both queued_this_component and
     *   queued_potential_components for each face within the component as we
     *   put it on the this_component queue.  And when we remove it, we set
     *   component.
     */
    Face_handle f0 = potential_components.front();
    potential_components.pop();

    assert(f0->info().queued_potential_components);
    if (f0->info().queued_this_component) {
      assert(f0->info().component >= 0);
      continue;
    }
    assert(f0->info().component < 0);

    ++component_ctr;

    std::queue<Face_handle> this_component;
    f0->info().queued_this_component = true;
    this_component.push(f0);

    while (!this_component.empty()) {
      Face_handle f1 = this_component.front();
      this_component.pop();

      assert(f1->info().queued_potential_components);
      assert(f1->info().queued_this_component);
      assert(f1->info().component < 0);

      f1->info().component = component_ctr;

      for (unsigned i=0; i<3; ++i) {
        Face_handle n = f1->neighbor(i);
        if (f1->is_constrained(i)) {
          if (! n->info().queued_potential_components) {
            assert(! n->info().queued_this_component);
            n->info().queued_potential_components = true;
            potential_components.push(n);
          } else {
            if (n->info().queued_this_component) { /* already handled, or it's part of this component also and has not been handled */
              if (n->info().component >= 0) {
                /* it has been handled.  If it's part of this component also, we do not have consistent sidedness. */
                if (n->info().component == component_ctr) {
                  consistent_sidedness = false;
                }
              }
            } else {
              /* not yet queued for this component.  It might be in a different component.*/
              assert(n->info().component < 0);
            }
          }
        } else {
          if (n->info().queued_this_component) {
            assert(n->info().queued_potential_components);
            assert(n->info().component < 0 || n->info().component == component_ctr);
          } else {
            n->info().queued_this_component = true;
            n->info().queued_potential_components = true;
            this_component.push(n);
          }
        }
      }
    }
  }
  max_component_ = component_ctr;
  DBG_FUNC_END(DBG_INPUT);
}

void
BasicTriangulation::
initialize(const BasicInput& input) {
  DBG_FUNC_BEGIN(DBG_INPUT);
  std::vector<Vertex_handle> ct_vertex_handles;
  ct_vertex_handles.reserve(input.vertices().size());

  /* Insert vertices.
   *
   * We store the handle of an incident face of the previously inserted vertex.
   * We use this as a location hint, on the assumption that vertex insert order
   * is somewhat spacially ordered.  This is only a hint to the CDT inserter.
   */
  if (input.vertices().size() == 0) {
    LOG(ERROR) << "Invalid input: no vertices.";
    exit(EXIT_INVALID_INPUT);
  }
  Face_handle prev_fh = NULL;
  for (size_t i=0; i<input.vertices().size(); ++i) {
    const auto p = input.vertices()[i];
    auto vh = insert(p.p, prev_fh);
    vh->info().original_vertex_idx = i;
    prev_fh = incident_faces(vh);
    ct_vertex_handles.push_back(vh);
  }
  if (number_of_vertices() != input.vertices().size()) {
    LOG(ERROR) << "Inserted only " << number_of_vertices() << " out of " << input.vertices().size() << " vertices.  Are the incident vertices?";
    exit(EXIT_INVALID_INPUT);
  }
  if (input.edges().size() == 0) {
    LOG(ERROR) << "Invalid input: no edges.";
    exit(EXIT_INVALID_INPUT);
  }
  /* Insert constraints.
   *
   * We insert constraints by providing the proper vertex handles from the CDT.
   * This provides a speedup of about 5 compared to adding the constraints by
   * coordinates.
   */
  for (const auto e : input.edges()) {
    if (ct_vertex_handles[e.u] == ct_vertex_handles[e.v]) {
      LOG(ERROR) << "Invalid input: Edge (" << e.u << ", " << e.v << ") is a loop.";
      exit(EXIT_INVALID_INPUT);
    }
    #ifdef HAVE_INTERSECTION_OF_CONSTRAINTS_EXCEPTION
    try {
    #endif
      insert_constraint(ct_vertex_handles[e.u], ct_vertex_handles[e.v]);
    #ifdef HAVE_INTERSECTION_OF_CONSTRAINTS_EXCEPTION
    } catch (Intersection_of_constraints_exception& err) {
      LOG(ERROR) << "Invalid input: Inserting edge (" << e.u << ", " << e.v << ") causes an intersection of constraints.";
      exit(EXIT_INVALID_INPUT);
    }
    #else
      #warning "Old CGAL version without Intersection_of_constraints_exception.  We will not detect some classes of invalid input."
    #endif
  }

  DBG(DBG_INPUT) << "Input has dimension " << dimension();
  DBG(DBG_INPUT) << "Have " << number_of_faces() << " finite faces";
  if (dimension() <= 1) {
    LOG(ERROR) << "Cannot handle input of dimension less than 2.";
    exit(EXIT_INVALID_INPUT);
  }
  assert(number_of_faces() > 0);
  tag_components( ct_vertex_handles[input.edges()[0].u], ct_vertex_handles[input.edges()[0].v] );

  DBG_FUNC_END(DBG_INPUT);
}
