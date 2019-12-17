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
#include "KineticTriangulation.h"
#include "EventQueue.h"
#include "WavefrontVertex.h"
#include "SkeletonStructure.h"

#include <tuple>
#include <unordered_map>

unsigned
KineticTriangulation::
get_num_initial_triangles(const BasicTriangulation& ct) {
  unsigned num_initial_triangles;
  if (restrict_component_ < 0) {
    num_initial_triangles = ct.tds().number_of_faces();
  } else {
    num_initial_triangles = 0;
    for (auto fit = ct.all_faces_begin(); fit != ct.all_faces_end(); ++fit) {
      num_initial_triangles += (fit->info().component == restrict_component_);
    }
  }
  return num_initial_triangles;
}

/** initialize the TDS, the Triangulation Data Structure
 *
 * After this function, we'll have Kinetic Triangles with neighborhood
 * references but no wavefront vertices yet.
 *
 * Given a basic constrained triangulation (CT) of the input (point and edge
 * set), start building up our kinetic triangulation.
 *
 * We begin with allocating sufficient memory to hold all wavefront vertices
 * and triangles during the entire straight-skeleton wavefront propagation.
 *
 * Next, we make kinetic triangles, one for each of the basic constrained
 * triangulation.  These don't have vertex references initially as the wavefront
 * vertices don't exist yet; those will come later.  The only exception is that
 * references to the infinite vertex are set already.  We do however, create a map
 * that associates a CT face with a kinetic triangle list index.
 *
 * Then, we set up neighborhood references between adjacent triangles that are
 * not seperated by a constraint.
 */
void
KineticTriangulation::
initialize_tds(
  const BasicInput& input,
  const BasicTriangulation& ct,
  const unsigned num_initial_triangles,
  FaceToTriangleIdxMap& face_to_triangle_idx,
  TriangleOriginalVertexIndexList& triangle_original_vertex_indices
) {
  DBG_FUNC_BEGIN(DBG_KT_SETUP);

  /* build kinetic triangulation */
  unsigned num_t = num_initial_triangles + input.get_num_extra_beveling_vertices();
  // at each input vertex we start one kinetic vertex per sector, and the number of sectors equals the vertex degree.
  // Additionally, we'll split vertices for beveling (including degree-1 vertices)

  unsigned num_events = num_t;
  auto num_v = input.get_total_degree() + 1 + num_events + input.get_num_extra_beveling_vertices();

  /** Allocate sufficient space for our arrays */
  vertices.reserve(num_v);
  triangles.reserve(num_t);
  face_to_triangle_idx.reserve(num_t);
  triangle_original_vertex_indices.reserve(num_t * 3);

  auto infinite_vertex_ct = ct.infinite_vertex();
  vertices.emplace_back(WavefrontVertex::make_infinite_vertex());
  const auto infinite = &vertices.back();

  /** Create kinetic triangles.  Without any vertices initially.
   * The only exception is that we already point to the correct infinite vertex.
   * Also set up the CT-face to KT index map.
   */
  DBG(DBG_KT_SETUP) << "#faces: " << num_t;
  for (auto fit = ct.all_faces_begin(); fit != ct.all_faces_end(); ++fit) {
    if (! fit->info().matches_component(restrict_component_)) continue;
    triangles.emplace_back( KineticTriangle(triangles.size(), restrict_component_ == -2 ? 0 : fit->info().component));
    assert(triangles.back().id == triangles.size() - 1);
    for (unsigned i=0; i<3; ++i) {
      triangle_original_vertex_indices.push_back(fit->vertex(i)->info().original_vertex_idx);
      if (fit->vertex(i) == infinite_vertex_ct) {
        triangles.back().set_vertex(i, infinite);
      }
    }
    assert(triangles.size()*3 == triangle_original_vertex_indices.size());
    face_to_triangle_idx.emplace(std::make_pair(fit, triangles.size()-1));
    DBG(DBG_KT_SETUP) << "Added " << &triangles.back()
      << "; input vidx: "
      << fit->vertex(0)->info().original_vertex_idx << ","
      << fit->vertex(1)->info().original_vertex_idx << ","
      << fit->vertex(2)->info().original_vertex_idx;
  }
  /** Set up neighborhood references between triangles that are not
   * seperated by a constraint.
   */
  for (auto fit = ct.all_faces_begin(); fit != ct.all_faces_end(); ++fit) {
    if (! fit->info().matches_component(restrict_component_)) continue;
    KineticTriangle *n[3];

    for (unsigned i=0; i<3; ++i) {
      n[i] = fit->is_constrained(i) ? NULL : &triangles[face_to_triangle_idx[fit->neighbor(i)]];
    }

    unsigned triangle_idx = face_to_triangle_idx[fit];
    KineticTriangle *t = &triangles[triangle_idx];
    DBG(DBG_KT_SETUP) << "set neighbors for " << t << " to "
      << n[0] << ", "
      << n[1] << ", "
      << n[2] << ".";
    t->set_neighbors(n[0], n[1], n[2]);
  }

  assert(triangles.size() == num_initial_triangles);
  DBG_FUNC_END(DBG_KT_SETUP);
}

/** Create wavefront edges for all constraints and references them from kinetic triangles.
 *
 * Also create straight skeleton (DCEL) faces and halfedge pairs for them all.
 */
void
KineticTriangulation::
create_supporting_lines(
  const BasicInput& input,
  const BasicTriangulation& ct,
  const unsigned num_initial_triangles,
  const FaceToTriangleIdxMap& face_to_triangle_idx,
  const TriangleOriginalVertexIndexList& triangle_original_vertex_indices
) {
  DBG_FUNC_BEGIN(DBG_KT_SETUP);
  /* total number of wavefront edges during the entire propragation.
   * this is an upper bound.
   *
   * num_t is a really rough upper bound on the number of split events.
   */
  unsigned num_t = num_initial_triangles + input.get_num_extra_beveling_vertices();
  auto num_wavefront_edges = input.edges().size()*2 + input.get_num_extra_beveling_vertices() + num_t*2;
  wavefront_edges->reserve(num_wavefront_edges);

  for (auto fit = ct.all_faces_begin(); fit != ct.all_faces_end(); ++fit) {
    if (! fit->info().matches_component(restrict_component_)) continue;
    WavefrontEdge *w[3];
    unsigned triangle_idx = face_to_triangle_idx.at(fit);
    KineticTriangle *t = &triangles[triangle_idx];

    for (unsigned i=0; i<3; ++i) {
      if (fit->is_constrained(i)) {
        unsigned vidxu = get_basic_vertex_idx_from_triangle_vertex_indices(input, triangle_original_vertex_indices, triangle_idx, ccw(i));
        unsigned vidxv = get_basic_vertex_idx_from_triangle_vertex_indices(input, triangle_original_vertex_indices, triangle_idx, cw (i));
        BasicVertex const * const u = &input.vertices()[vidxu];
        BasicVertex const * const v = &input.vertices()[vidxv];
        if (!input.has_edge(vidxu, vidxv)) {
          LOG(ERROR) << "Cannot find edge for triangulation constraint (" << vidxu << ", " << vidxv << ") in input.  Input is not a PSLG.";
          LOG(ERROR) << " v" << vidxu << ": " << CGAL_point(input.vertices()[vidxu].p);
          LOG(ERROR) << " v" << vidxv << ": " << CGAL_point(input.vertices()[vidxv].p);
          LOG(ERROR) << " Probably one of these points is on the interior of another edge.";
          exit(1);
        }
        auto const & edge = input.get_edge(vidxu, vidxv);

        DBG(DBG_KT_SETUP) << "at " << t << "/" << i << ": vertices are " << vidxu << " and " << vidxv;

        WavefrontEdge* buddy_wavefront_edge = NULL;

        BasicTriangulation::Face_handle ct_n_hdl = fit->neighbor(i);
        if (face_to_triangle_idx.find(ct_n_hdl) == face_to_triangle_idx.end()) {
          /* neighbor not in map */
          assert(restrict_component_ >= 0);
          assert(! ct_n_hdl->info().matches_component(restrict_component_) );
        } else {
          KineticTriangle &n = triangles[face_to_triangle_idx.at(ct_n_hdl)];
          assert( ct_n_hdl->info().matches_component(restrict_component_) );
          unsigned idx_in_n = ct_n_hdl->index(fit);
          buddy_wavefront_edge = n.wavefront(idx_in_n); /* may still be null, and that's OK */
        }

        SkeletonDCELFace* skeleton_dcel_face = skeleton.setup_new_input_edge(buddy_wavefront_edge /* may be NULL and that's OK */);

        wavefront_edges->emplace_back(WavefrontEdge(u->p, v->p, edge.weight, t, skeleton_dcel_face));
        w[i] = &wavefront_edges->back();
      } else {
        w[i] = NULL;
      }
    }

    t->set_wavefronts(w[0], w[1], w[2]);
  }
  DBG_FUNC_END(DBG_KT_SETUP);
}


/** Return a BasicVertex index from a triangle index and vertex position within the triangle.
 *
 * This only works for triangles that were part of the initial triangulation
 * and where we set up the mapping in triangle_original_vertex_indices.
 *
 * So it gets invalidated partially as soon as we start splitting triangles.
 *
 * This function primary checks bounds and some basic validity.
 */
unsigned
KineticTriangulation::
get_basic_vertex_idx_from_triangle_vertex_indices(
  const BasicInput& input,
  const TriangleOriginalVertexIndexList& triangle_original_vertex_indices,
  unsigned t_idx,
  unsigned i
) {
  assert(i < 3);

  /* Make sure we do not overrun the original triangulation size.
   */
  unsigned idx = t_idx * 3 + i;
  assert(idx < triangle_original_vertex_indices.size());
  assert(triangle_original_vertex_indices[idx] >= 0);
  unsigned vidx = triangle_original_vertex_indices[idx];
  assert(vidx < input.vertices().size());
  return vidx;
}

/** Return a BasicVertex from a triangle index and vertex position within the triangle.
 *
 * See get_basic_vertex_idx_from_triangle_vertex_indices() for details.
 */
const BasicVertex&
KineticTriangulation::
get_basic_vertex_from_triangle_vertex_indices(
  const BasicInput& input,
  const TriangleOriginalVertexIndexList& triangle_original_vertex_indices,
  unsigned t_idx,
  unsigned i
) {
  const BasicVertex & bv = input.vertices()[ get_basic_vertex_idx_from_triangle_vertex_indices(input, triangle_original_vertex_indices, t_idx, i) ];
  return bv;
}

/** mark the original vertex as unusable.
 *
 * once we have created a kinetic vertex for a basic (input) vertex,
 * we no longer want to touch the basic vertex from input.vertices().
 *
 * This function marks the entry in the triangle-vertex to original-vertex map
 * as invalid, preventing further accesses.
 */
void
KineticTriangulation::
invalidate_basic_vertex_idx_in_triangle_vertex_indices(
  const BasicInput& input,
  TriangleOriginalVertexIndexList& triangle_original_vertex_indices,
  unsigned t_idx,
  unsigned i
) {
  unsigned idx = t_idx * 3 + i;
  assert(idx < triangle_original_vertex_indices.size());
  assert(triangle_original_vertex_indices[idx] >= 0);
  triangle_original_vertex_indices[idx] = -1;
}

/** Create wavefront vertices, except those that need beveling (such as degree-1 vertices)
 */
void
KineticTriangulation::
create_kinetic_vertices(
  const BasicInput& input,
  const BasicTriangulation& ct,
  TriangleOriginalVertexIndexList& triangle_original_vertex_indices
) {
  DBG_FUNC_BEGIN(DBG_KT_SETUP);

  /* We iterate over all triangles, and in each triangle over each
   * vertex v.  If the triangle is the start of a clockwise triangle
   * fan about v, that is, if the correct edge incident to v is contraint,
   * we set up the kinetic vertex.
   */
  for (auto t_it = triangles.begin(); t_it != triangles.end(); ++t_it) {
    /* Invariant: all previous triangles have vertices set up on
     * the "head" side of each constraint they have.
     *
     * They not necessarily have vertices set up on the "tail" side of
     * each constraint or at vertices with no incident (in this triangle)
     * constraint.
     */
    unsigned t_idx = t_it->id;
    assert(t_idx == t_it - triangles.begin());
    DBG(DBG_KT_SETUP) << "setting up kinetic vertices for " << &*t_it;
    for (unsigned i=0; i<3; ++i) {
      DBG(DBG_KT_SETUP) << "vertex at idx " << i;
      if (! t_it->is_constrained(ccw(i))) continue;

      /* the edge cw of i (opposite i's ccw vertex) is constrained. */
      AroundVertexIterator faces_it = incident_faces_iterator(&*t_it, i);
      assert(! faces_it.next_triangle_ccw() );
      AroundVertexIterator most_cw_triangle = faces_it.most_cw();

      WavefrontEdge *l = t_it->wavefront(ccw(i));
      WavefrontEdge *r = most_cw_triangle.t()->wavefront(cw(most_cw_triangle.v_in_t_idx()));

      const BasicVertex& bv = get_basic_vertex_from_triangle_vertex_indices(input, triangle_original_vertex_indices, t_idx, i);

      if (bv.degree == 1) {
        continue; // Need to split/bevel in any case.
      }

      assert(bv.degree > 1);

      // XXX do not make a vertex if this is reflex and the reflex_beveling_add is greater than 1.
      WavefrontVertex* v = vertices.make_initial_vertex(bv.p, l, r);

      // Should not have created a vertex that needed beveling.
      assert(bv.reflex_beveling_add == 0 || v->is_convex_or_straight());

      for (AroundVertexIterator it = incident_faces_iterator(&*t_it, i); it != incident_faces_end(); ++it) {
        (*it).set_vertex(it.v_in_t_idx(), v);
        invalidate_basic_vertex_idx_in_triangle_vertex_indices(input, triangle_original_vertex_indices, (*it).id, it.v_in_t_idx());
        DBG(DBG_KT_SETUP) << "  setting vertex to " << v << " in " << &*it << "(" << it.v_in_t_idx() << ")";
      };

      DBG(DBG_KT_SETUP) << "  setting vertex 1 to " << v << " in " << *l;
      DBG(DBG_KT_SETUP) << "  setting vertex 0 to " << v << " in " << *r;
      assert(l->vertex(1) == v);
      assert(r->vertex(0) == v);
    }
  }
  DBG_FUNC_END(DBG_KT_SETUP);
}



/** Split vertex
 *
 * For creating bevels (such as when dealing with degree-1 vertices),
 * we need to split triangulation vertices.
 *
 * This function splits a vertex v, as given by a triangle t and vertex index.
 * The triangle is duplicated, and the new triangle is ccw of t at v.
 *
 * Returns a pointer to the new KineticTriangle.  The new edges is between
 * vertices 0 and 1.
 */
KineticTriangle *
KineticTriangulation::
split_vertex(
  KineticTriangle *t,
  unsigned i,
  WavefrontEdge * new_e,
  WavefrontVertex * new_v
) {
  DBG_FUNC_BEGIN(DBG_KT_SETUP);
  assert(new_e);
  assert(new_v);
  assert(t);

  /* Split the triangle */
  triangles.emplace_back(KineticTriangle(triangles.size(), t->component));
  assert(triangles.back().id == triangles.size() - 1);
  KineticTriangle& new_t = triangles.back();
  DBG(DBG_KT_SETUP) << " New triangle " << &new_t;

  new_t.wavefronts[0] = NULL;
  new_t.wavefronts[1] = new_e;
  new_t.wavefronts[2] = t->wavefronts[ cw (i) ];
  new_t.neighbors [0] = t;
  new_t.neighbors [1] = NULL;
  new_t.neighbors [2] = t->neighbors[ cw (i) ];

  assert(new_t.vertices[1] == NULL);
  assert(new_t.vertices[2] == NULL);
  new_t.set_vertex(0, new_v);
  if (t->vertices[ ccw(i) ]) { new_t.set_vertex(1, t->vertices[ ccw(i) ]); }
  if (t->vertices[     i  ]) { new_t.set_vertex(2, t->vertices[     i  ]); }
  DBG(DBG_KT_SETUP) << "  setting vertex to " << new_v << " in " << &new_t << "(" << 0 << ")";

  t->wavefronts[ cw (i) ] = NULL;
  t->neighbors [ cw (i) ] = &new_t;

  assert(!new_t.wavefronts[2] != !new_t.neighbors[2]);
  if (new_t.wavefronts[2]) {
    new_t.wavefronts[2]->set_incident_triangle(&new_t);
    DBG(DBG_KT_SETUP) << "  setting incident triangle for wavefronts[2], " << *new_t.wavefronts[2];
  } else {
    unsigned pos = new_t.neighbors[2]->index(t);
    new_t.neighbors[2]->set_neighbor(pos, &new_t);
    DBG(DBG_KT_SETUP) << "  setting neighbor triangle for neighbors[2], " << new_t.neighbors[2];
  }

  if (new_e->incident_triangle() == NULL) {
    new_e->set_incident_triangle(&new_t);
      DBG(DBG_KT_SETUP) << "  setting incident triangle for current_edge, " << *new_e;
  }

  DBG(DBG_KT_SETUP) << " Old triangle details: " << t;
  DBG(DBG_KT_SETUP) << " New triangle details: " << &new_t;

  DBG_FUNC_END(DBG_KT_SETUP);
  return &new_t;
}

/** return (position, is_infinite) of vertex i of triangle t.
 *
 * This uses the kinetic triangulation's vertex location
 * if already set, and falls back to using the vertex
 * position from the original, underlying, constrained triangulation.
 *
 * The latter only works for triangles that were created initially,
 * not any that are the result of splits (but those should all have
 * kinetic vertices yet).  NO, WRONG, XXX NOT GUARANTEED.
 */
std::tuple<Point_2, bool>
KineticTriangulation::
get_vertex_pos(const BasicInput& input,
  const TriangleOriginalVertexIndexList& triangle_original_vertex_indices,
  const KineticTriangle * t,
  unsigned i
) {
  //DBG_FUNC_BEGIN(DBG_KT_SETUP);
  DBG_INDENT_INC();

  WavefrontVertex const * const kv = t->vertex(i);

  Point_2 pos;
  bool is_inf = false;

  if (kv) {
    DBG(DBG_KT_SETUP) << "  v is " << kv;
    if (kv->is_infinite) { /* we already have a vertex set at t's cw */
      is_inf = true;
    } else {
      pos = kv->pos_zero;
    }
  } else {
    DBG(DBG_KT_SETUP) << "  kinetic vertex is not yet set up, using vertex from input triangulation.";
    /* The infinite vertex is associated in kinetic triangles already
     * in initialize_tds at the very start of setting things up.
     * So if we are here, it's not the infinite vertex.
     *
     * However, it may be a vertex we'll have to bevel later, so it might
     * still not be set in the kinetic triangulation.  In that case,
     * get its position from the underlying original triangulation and input.
     */
    assert(t->id < triangle_original_vertex_indices.size()/3); // XXX -- check if this holds
    const BasicVertex& bv = get_basic_vertex_from_triangle_vertex_indices(input, triangle_original_vertex_indices, t->id, i);
    pos = bv.p;
  }

  DBG_INDENT_DEC();
  //DBG_FUNC_END(DBG_KT_SETUP);
  return std::make_tuple(pos, is_inf);
}

/** Create bevels at one vertex.
 *
 * .) find the neighboring constraints,
 * .) create beveling wavefront supporting lines
 * .) create the kinetic vertices between adjacent wavefronts, and
 * .) split one or more triangles and set the vertices accordingly.
 */
void
KineticTriangulation::
create_bevels_at_vertex(
  const BasicInput& input,
  const BasicTriangulation& ct,
  const TriangleOriginalVertexIndexList& triangle_original_vertex_indices,
  KineticTriangle *t,
  unsigned i
) {
  DBG_FUNC_BEGIN(DBG_KT_SETUP);
  /* We should not have any triangles with undefined vertices after
   * we went past the original size.  In particular, since the
   * triangle_vertex_handle_idx array is not defined for the new ones,
   * and in fact may have become invalid for some of the existing entries
   * too.
   */
  DBG(DBG_KT_SETUP) << t << "; vertex " << i;
  WavefrontEdge *l, *r;

  AroundVertexIterator most_ccw_triangle = incident_faces_iterator(t, i).most_ccw();
  AroundVertexIterator most_cw_triangle  = incident_faces_iterator(t, i).most_cw();
  l = most_ccw_triangle.t()->wavefront(ccw(most_ccw_triangle.v_in_t_idx()));
  r = most_cw_triangle .t()->wavefront(cw (most_cw_triangle .v_in_t_idx()));

  DBG(DBG_KT_SETUP) << " incident edges: " << *l << ", " << *r;

  /* Build the list of wavefront edges */
  /*************************************/
  std::vector<WavefrontEdge *> edges;
  edges.push_back(l);
  const BasicVertex& bv = get_basic_vertex_from_triangle_vertex_indices(input, triangle_original_vertex_indices, t->id, i);

  if (bv.degree == 1) {
    if (bv.reflex_beveling_add >= 2) {
      NOTIMPL_MSG << "Beveling degree-one vertex with 2 or more extra vertices not implemented yet";
      assert(false);  // Beveling reflex vertex
      abort();
    } {
      assert(l->l()->l == r->l()->l.opposite());

      if (!compare_NT_real_eq(l->l()->weight, r->l()->weight)) {
        NOTIMPL_MSG << "Unclear what to do when the incidents weights do not match in beveling";
        assert(false);  // Beveling reflex vertex
        abort();
      }

      wavefront_edges->emplace_back(
        WavefrontEdge( std::make_shared<const WavefrontSupportingLine>(l->l()->l.opposite().perpendicular(bv.p), l->l()->weight), NULL )
      );
      WavefrontEdge *w = &wavefront_edges->back();
      edges.push_back(w);
      DBG(DBG_KT_SETUP) << " New wavefront edge: " << *w;
    }
  } else {
    NOTIMPL_MSG << "Beveling reflex vertices not implemented yet";
    assert(false);  // Beveling reflex vertex
    abort();
  }
  edges.push_back(r);

  std::vector<WavefrontVertex *> new_wavefront_vertices;
  assert(edges.size() > 2);

  DBG(DBG_KT_SETUP) << " Edges:";
  for (auto& e: edges) {
    DBG(DBG_KT_SETUP) << "  - " << *e;
  }

  /* Build the list of wavefront vertices */
  /***************************************/
  auto current_edge = edges.begin();
  auto previous_edge = current_edge;
  WavefrontVertex* prev_vertex = NULL;
  DBG(DBG_KT_SETUP) << " New wavefront vertices:";
  for (++current_edge; current_edge != edges.end(); previous_edge = current_edge++) {
    WavefrontVertex* v = vertices.make_initial_vertex(bv.p, *previous_edge, *current_edge, true);
    if (prev_vertex) {
      v->link_tail_to_tail(prev_vertex);
    }
    prev_vertex = v;

    new_wavefront_vertices.push_back(v);
    DBG(DBG_KT_SETUP) << "  - " << v;
  }

  /* And update the triangulation, splitting vertices */
  /****************************************************/
  auto new_v = new_wavefront_vertices.begin();
  current_edge = edges.begin();
  ++current_edge;
  auto t_it = most_ccw_triangle;
  while (t_it != incident_faces_end()) {
    unsigned t_idx = t_it.t()->id;
    assert(t_idx == t_it.t() - &(*triangles.begin()));

    /* if we split in the iteration before, this is a duplicate (as we set it in split_vertex()), but who cares :) */
    t_it.t()->set_vertex(t_it.v_in_t_idx(), *new_v);
    DBG(DBG_KT_SETUP) << " Setting vertex to " << (*new_v) << " in " << t_it.t() << "(" << t_it.v_in_t_idx() << ")";

    /* If the normal of the right edge points into the current triangle,
     * we move to the next wavefront/vertex.  We split the triangle by
     * duplicating the triangulation vertex here, assigning the previous
     * and new wavefront vertex to each of the triangulation vertices from
     * the split.
     */
    bool split_this;
    bool need_flip = false;
    assert(*current_edge == (*new_v)->incident_wavefront_edge(1));

    if (&(*new_v) == &(new_wavefront_vertices.back())) {
      DBG(DBG_KT_SETUP) << "  At last vertex, nothing to split anymore";
      split_this = false;
    } else if (t_it.next_triangle_cw() == NULL) {
      DBG(DBG_KT_SETUP) << "  At last triangle, we need to split here.";
      split_this = true;
    } else {
      DBG(DBG_KT_SETUP) << " Checking if we need to split in " << t_it.t();
      Point_2 pos_t_ccw;
      bool infinite_ccw_vertex;
      std::tie(pos_t_ccw, infinite_ccw_vertex) = get_vertex_pos(input, triangle_original_vertex_indices, t_it.t(), ccw(t_it.v_in_t_idx()));

      if (! infinite_ccw_vertex) {
        DBG(DBG_KT_SETUP) << "  All nice and finite";
        Point_2 pos_plus_normal(bv.p + (*current_edge)->l()->normal_direction.perpendicular(CGAL::RIGHT_TURN));

        DBG(DBG_KT_SETUP) << "   p     " << bv.p;
        DBG(DBG_KT_SETUP) << "   p+    " << pos_plus_normal;
        DBG(DBG_KT_SETUP) << "   ptccw " << pos_t_ccw;
        auto orientation = CGAL::orientation(bv.p, pos_plus_normal, pos_t_ccw);
        DBG(DBG_KT_SETUP) << "   orientation " << orientation;
        split_this = (orientation == CGAL::LEFT_TURN);
      } else {
        DBG(DBG_KT_SETUP) << "  Unbounded triangle: ccw vertex is the infinite vertex";
        //WavefrontVertex const * const cw_v = t_it.t()->vertex(cw(t_it.v_in_t_idx()));
        //DBG(DBG_KT_SETUP) << "   cw_vertex point              " << cw_v;

        auto next_t = t_it;
        ++next_t;
        Point_2 next_pos_t_ccw;
        bool next_infinite_ccw_vertex;
        std::tie(next_pos_t_ccw, next_infinite_ccw_vertex) = get_vertex_pos(input, triangle_original_vertex_indices, next_t.t(), ccw(next_t.v_in_t_idx()));
        assert(!next_infinite_ccw_vertex);

        DBG(DBG_KT_SETUP) << "   current_edge supporting line " << CGAL_line((*current_edge)->l()->l);
        DBG(DBG_KT_SETUP) << "   next_pos_t_ccw " << next_pos_t_ccw;

        /* We can err on the side of having triangulation spokes on the CH boundary,
         * or we can have collinear points on the CH boundary.
         * Whichever we pick, we will then have to figure out what the right way
         * to flip is in one case or another.
         */
        // split_this = (*current_edge)->l()->l.has_on_negative_side(next_pos_t_ccw);
        split_this = ! (*current_edge)->l()->l.has_on_positive_side(next_pos_t_ccw);
        if (split_this) {
          Point_2 pos_t_cw;
          bool infinite_cw_vertex;
          std::tie(pos_t_cw, infinite_cw_vertex) = get_vertex_pos(input, triangle_original_vertex_indices, t_it.t(), cw(t_it.v_in_t_idx()));
          assert(!infinite_cw_vertex);

          DBG(DBG_KT_SETUP) << "   has on pos " << (*current_edge)->l()->l.has_on_positive_side(pos_t_cw);
          DBG(DBG_KT_SETUP) << "   has on neg " << (*current_edge)->l()->l.has_on_negative_side(pos_t_cw);

          /* The triangle created by our splitting the edge from v to ccw(v) will be incident
           * to the infinite vertex, but the old vertex will not be on the convex hull,
           * so we need to split things.
           */
          need_flip = (*current_edge)->l()->l.has_on_positive_side(pos_t_cw);
          DBG(DBG_KT_SETUP) << "   need flip " << need_flip;
        }
      }
    }

    if (split_this) {
      DBG(DBG_KT_SETUP) << " Splitting triangle " << t_it.t();
      assert(t_it.t()->vertex(t_it.v_in_t_idx()) == *new_v);
      ++new_v;
      DBG(DBG_KT_SETUP) << "  next vertex is " << (*new_v);

      KineticTriangle * new_t = split_vertex(t_it.t(), t_it.v_in_t_idx(), *current_edge, *new_v);
      assert((*current_edge)->incident_triangle() == new_t || &(*current_edge) == &(edges.back()));

      if (need_flip) {
        DBG(DBG_KT_SETUP) << " need flip.";
        DBG(DBG_KT_SETUP) << "  t  " << t_it.t();
        DBG(DBG_KT_SETUP) << "  tn " << new_t;
        LOG(WARNING) << __FILE__ << ":" << __LINE__ << " " << "untested code path.";
        new_t->do_raw_flip_inner(0);
        DBG(DBG_KT_SETUP) << "  t  " << t_it.t();
        DBG(DBG_KT_SETUP) << "  tn " << new_t;
      };

      t_it = AroundVertexIterator(new_t, 0);
      ++current_edge;
    } else {
      DBG(DBG_KT_SETUP) << " Moving on, not splitting this triangle " << t_it.t();
      ++t_it;
    }
  }
  assert(&(*new_v) == &(new_wavefront_vertices.back()));
  DBG_FUNC_END(DBG_KT_SETUP);
}


/** Create bevels
 *
 * Split vertices for bevels at degree-1 vertices and where we want other reflex vertices to bevel
 *
 * For each vertex in the triangulation that is not yet initialized, call create_bevels_at_vertex()
 */
void
KineticTriangulation::
create_bevels(
  const BasicInput& input,
  const BasicTriangulation& ct,
  const TriangleOriginalVertexIndexList& triangle_original_vertex_indices
) {
  DBG_FUNC_BEGIN(DBG_KT_SETUP);

  unsigned initial_triangles_size = triangles.size();

  /* verify all existing wavefronts have an incident triangle that
   * reference back to the wavefront.
   */
  for (auto &wf : *wavefront_edges) {
    assert(wf.incident_triangle());
    assert(wf.incident_triangle()->has_wavefront(&wf));
  }


  /* set up bevels */
  for (auto t_it = triangles.begin(); t_it != triangles.end(); ++t_it) {
    assert(t_it->id == t_it - triangles.begin());

    for (unsigned i=0; i<3; ++i) {
      if (t_it->vertex(i) != NULL) continue;
      assert(t_it->id < initial_triangles_size);

      DBG(DBG_KT_SETUP) << "Doing bevels at t " << &*t_it << "; vertex " << i;
      create_bevels_at_vertex(input, ct, triangle_original_vertex_indices, &*t_it, i);
    }
  }

  /* assert all kinetic triangulation vertices have an assigned
   * wavefront vertex.
   */
  for (auto t_it = triangles.begin(); t_it != triangles.end(); ++t_it) {
    unsigned t_idx = t_it->id;
    assert(t_idx == t_it - triangles.begin());
    for (unsigned i=0; i<3; ++i) {
      assert(t_it->vertex(i) != NULL);
    }
  }

  /* verify all existing wavefronts have an incident triangle that
   * reference back to the wavefront.
   */
  for (auto &wf : *wavefront_edges) {
    DBG(DBG_KT_SETUP) << " wf: " << wf;
    assert(wf.incident_triangle());
    assert(wf.incident_triangle()->has_wavefront(&wf));
  }

  DBG_FUNC_END(DBG_KT_SETUP);
}

void
KineticTriangulation::
store_initial_wavefront_vertices() {
  for (auto& e : *wavefront_edges) {
    assert(e.is_initial);
    e.set_initial_vertices();
  }
}

void
KineticTriangulation::
link_dcel_halfedges_on_ignored_side() {
  DBG_FUNC_BEGIN(DBG_SKEL);

  for (auto he = skeleton.halfedges_begin(); he != skeleton.halfedges_end(); ++he) {
    if (! he->is_emanating_input() && he->opposite()->is_emanating_input()) {
      /* This is the "ignored" side of an input edge. */
      if (he->next() != NULL) { // already processed
        continue;
      }

      SkeletonDCELFace* f;
      SkeletonDCELCbb* c;
      f = skeleton.new_face();
      c = skeleton.new_outer_ccb();
      c->set_face(f);
      auto cur_he = he;

      do {
        /* find next */
        auto next = cur_he->opposite();
        while (NULL != next->prev()) {
          next = next->prev()->opposite();
          assert(next);
        }
        cur_he->set_next(next);
        cur_he = next;
      } while (cur_he != he);

      f->add_outer_ccb(c, &*he);
    }
  }
  DBG_FUNC_END(DBG_SKEL);
}

/** Set up DCEL half-edges around one straight skeleton face
 *
 * The kinetic vertex start is the "right" vertex of base, which traces out f.  We are walking
 * along the boundary of f counter-clockwise.
 */
SkeletonDCELHalfedge *
KineticTriangulation::
create_remaining_skeleton_dcel_one_face(WavefrontVertex* start, SkeletonDCELHalfedge* base, SkeletonDCELCbb* c, SkeletonDCELFace* f) {
  DBG_FUNC_BEGIN(DBG_SKEL);
  assert(start->is_initial);
  assert(c);
  assert(f);

  if (base) {
    DBG(DBG_SKEL) << "starting at : " << start << "; base: " << *base;
  } else {
    DBG(DBG_SKEL) << "starting at : " << start << "; base: -";
  }
  SkeletonDCELHalfedge* prev_he = base;
  WavefrontVertex* cur_v = start;
  WavefrontVertex* prev_v;
  int side_of_f = 0;
  while (1) {
    DBG(DBG_SKEL) << " doing " << cur_v << "; " << cur_v->details();
    assert(!cur_v->is_degenerate());

    SkeletonDCELHalfedge* new_he;
    /* Make a new halfedge or use the one we already have for this vertex. */
    assert(! cur_v->skeleton_dcel_halfedge(side_of_f) );
    if (cur_v->skeleton_dcel_halfedge(1-side_of_f) ) {
      new_he = cur_v->skeleton_dcel_halfedge(1-side_of_f)->opposite();
      assert(new_he->is_on_outer_ccb());
      assert(new_he->outer_ccb() == NULL);
    } else {
      new_he = skeleton.new_edge();
    }
    new_he->set_outer_ccb(c);
    DBG(DBG_SKEL) << "  he for " << cur_v << ": " << *new_he;

    if (prev_he) {
      assert(prev_he->next() == NULL);
      new_he->set_prev(prev_he);
    }
    prev_he = new_he;
    cur_v->set_skeleton_dcel_halfedge(side_of_f, new_he);


    DBG(DBG_SKEL) << "  looking for next. ";
    do {
      prev_v = cur_v;
      /* if side_of_f is 1 (i.e. if the face is to the right of us, we are
       * going forward on this vertex, else we are going backwards. */
      if (side_of_f) {
        cur_v = cur_v->prev_vertex(side_of_f);
      } else {
        cur_v = cur_v->next_vertex(side_of_f);
      }
      DBG(DBG_SKEL) << "  considering " << cur_v;

      if (cur_v == NULL && side_of_f == 0) { /* prev wavefront vertex escapes to infinity */
        DBG(DBG_SKEL) << "  went to infinity (and beyond?).  jumping over using wavefront edge";
        assert(!prev_v->has_stopped());
        const WavefrontEdge * const e = prev_v->incident_wavefront_edge(0);
        assert(!e->is_dead());
        assert(e->vertex(1) == prev_v);
        cur_v = e->vertex(0);
        side_of_f = 1;
        goto foundvertex;
      } else if (cur_v == NULL && side_of_f == 1) {
        DBG(DBG_SKEL) << "  done now, arrived at base.";
        assert(prev_v->is_initial);
        goto looped_around;
      } else if (cur_v == start) {
        DBG(DBG_SKEL) << "  done now, looped around";
        assert(cur_v->prev_vertex(0) == prev_v);
        side_of_f = 0;
        assert(prev_v->is_initial);
        assert(cur_v->is_beveling || prev_v->is_beveling);
        goto looped_around;
      } else {
        assert(cur_v);

        /* Figure out which side f is one */
        if (cur_v->prev_vertex(0) == prev_v) {
          side_of_f = 0;
        } else {
          assert(cur_v->next_vertex(1) == prev_v);
          side_of_f = 1;
        }
      }
    } while (cur_v->is_degenerate());
  foundvertex:
    DBG(DBG_SKEL) << "  found " << cur_v;
  };

looped_around:
  if (cur_v) {
    DBG(DBG_SKEL) << "back at start, so this is the result of beveling";
    /* beveling vertex, this is the start vertex, again, hopefully. */
    assert(cur_v == start);
    assert(side_of_f == 0);
    assert(cur_v->skeleton_dcel_halfedge(side_of_f));
    cur_v->skeleton_dcel_halfedge(side_of_f)->set_prev(prev_he);
  } else {
    assert(base->prev() == NULL);
    base->set_prev(prev_he);
  }
  DBG_FUNC_END(DBG_SKEL);
  return prev_he;
}

/** Create a dcel vertex and set it at all halfedges incident to that vertex.
 */
void
KineticTriangulation::
set_dcel_vertex(SkeletonDCELHalfedge* start, const Point_2* p, const NT& time) {
  SkeletonDCELVertex* new_v = skeleton.new_vertex();
  new_v->set_halfedge(start);
  if (p) {
    Point_3 *pp = skeleton.new_point( Point_3(p->x(), p->y(), time) );
    new_v->set_point(pp);
  }

  unsigned degree = 0;
  SkeletonDCELHalfedge* he = start;
  do {
    he->set_vertex(new_v);
    DBG(DBG_SKEL) << "  Setting vertex for " << *he;
    assert(he->next());
    he = he->next()->opposite();
    degree++;
  } while (he != start);
}

/** build the remaining skeleton dcel
 *
 * For each "initial" vertex, build the structure of its right-hand face, going
 * around it in a ccw direction.  Its corresponding dcel face already exists if
 * the face is traced out by an input edge.  For edges that are the result of
 * beveling, we need to create the dcel face (and the cbb) first.
 *
 * At each kinetic vertex in this walk, if it's not degenerate (i.e. if it has
 * length > 0), create a halfedge pair unless we already have it.  Then link up
 * the correct halfedge with its predecessor in the list, and set the
 * appropriate dcel halfedge in the kinetic vertex.
 *
 * may only be called once, at the end.
 */
void
KineticTriangulation::
create_remaining_skeleton_dcel() {
  DBG_FUNC_BEGIN(DBG_SKEL);
  assert(!finalized);

  /* set up halfedges */
  auto v_it = vertices.begin();
  for (; v_it != vertices.end(); ++v_it) {
    WavefrontVertex* v = &(*v_it);
    bool new_face;
    SkeletonDCELFace* f;
    SkeletonDCELCbb* c;
    SkeletonDCELHalfedge* base;
    DBG(DBG_SKEL) << " v: " << v;

    if (v->is_infinite) continue;
    if (!v->is_initial) break;
    if (v->is_degenerate()) continue;

    assert(!v->is_degenerate());
    assert(v->incident_wavefront_edge(0));
    if (NULL != (f = v->incident_wavefront_edge(0)->skeleton_face)) {
      new_face = false;

      assert(f->outer_ccbs_begin() != f->outer_ccbs_end());
      base = * f->outer_ccbs_begin();
      c = base->outer_ccb();
    } else {
      /* the result of beveling */
      new_face = true;

      f = skeleton.new_face();
      c = skeleton.new_outer_ccb();
      c->set_face(f);
      base = NULL;
    };
    DBG(DBG_SKEL) << "new_face: " << new_face;
    SkeletonDCELHalfedge* last_he = create_remaining_skeleton_dcel_one_face(v, base, c, f);
    if (new_face) {
      assert(!base);
      f->add_outer_ccb(c, last_he);
    }
    DBG(DBG_SKEL) << "skeleton_face: " << *f;
    if (!base) {
      base = *f->outer_ccbs_begin();
    }
    DBG(DBG_SKEL) << "base         : " << *base;
    for (SkeletonDCELHalfedge *he = base->next(); he != base; he = he->next()) {
      DBG(DBG_SKEL) << " he        : " << *he;
    }
  }
  for (; v_it != vertices.end(); ++v_it) {
    assert(!v_it->is_initial);
  };

  if (restrict_component_ >= 0) {
    link_dcel_halfedges_on_ignored_side();
  }

  for (const auto& v : vertices) {
    assert(v.is_infinite + v.is_degenerate() == 1 || !!v.skeleton_dcel_halfedge(0));
    assert(v.is_infinite + v.is_degenerate() == 1 || !!v.skeleton_dcel_halfedge(1));
  }

  /* set up vertices */
  bool have_infinite_vertex;
  if (restrict_component_ >= 0) {
    have_infinite_vertex = false;
    for (const auto& v : vertices) {
      if (v.is_infinite) {
        continue;
      }
      if (! v.has_stopped()) {
        have_infinite_vertex = true;
        break;
      }
    }
    DBG(DBG_SKEL) << " have an infinite vertex: " << have_infinite_vertex;
    if (!have_infinite_vertex) {
      skeleton.set_num_v_skew(0);
    };
  } else {
    have_infinite_vertex = true;
  }
  skeleton.set_number_of_points_and_curves();

  DEBUG_STMT(bool did_infinite_vertex = false);
  for (const auto& v : vertices) {
    if (v.is_infinite || v.is_degenerate()) {
      continue;
    };
    DBG(DBG_SKEL) << "  Setting dcel vertex for " << v.details();
    if (v.is_initial) {
      SkeletonDCELHalfedge* he = v.skeleton_dcel_halfedge(1);
      DBG(DBG_SKEL) << "  he (initial): " << *he;
      if (!he->vertex()) {
        assert(v.time_start == CORE_ZERO);
        set_dcel_vertex(he, &v.pos_start, CORE_ZERO);
      }
    }

    SkeletonDCELHalfedge* he = v.skeleton_dcel_halfedge(0);
    DBG(DBG_SKEL) << "  he: " << *he;
    if (!he->vertex()) {
      if (v.has_stopped()) {
        set_dcel_vertex(he, &v.pos_stop(), v.time_stop());
      } else {
        assert(!did_infinite_vertex);
        DEBUG_STMT(did_infinite_vertex = true);
        set_dcel_vertex(he, NULL, CORE_ZERO);
      }
    }
  }
  assert(did_infinite_vertex ^ !have_infinite_vertex);

  /* set up segments/rays for arcs */
  for (const auto& v : vertices) {
    if (v.is_infinite || v.is_degenerate()) {
      continue;
    };
    DBG(DBG_SKEL) << "  at v: " << &v;

    SkeletonDCELHalfedge* he0 = v.skeleton_dcel_halfedge(0);
    SkeletonDCELHalfedge* he1 = v.skeleton_dcel_halfedge(1);

    DBG(DBG_SKEL) << "   he0: " << *he0;
    DBG(DBG_SKEL) << "   he1: " << *he1;

    SkeletonDCELHalfedge::X_monotone_curve * p;
    if (v.has_stopped()) {
      assert( ! he0->vertex()->has_null_point() );
      assert( ! he1->vertex()->has_null_point() );
      p = skeleton.new_segment( Segment_3(he0->vertex()->point(), he1->vertex()->point()) );
    } else {
      assert(   he0->vertex()->has_null_point() );
      assert( ! he1->vertex()->has_null_point() );
      Vector_3 vec(v.velocity.x(), v.velocity.y(), CORE_ONE);
      p = skeleton.new_ray( Ray_3(he1->vertex()->point(), vec) );
    }
    he0->set_curve(p);
    he1->set_curve(p);
  }
  /* and input segments */
  for (auto he = skeleton.halfedges_begin(); he != skeleton.halfedges_end(); ++he) {
    if (he->is_emanating_input()) {
      if (he->has_null_curve()) {
        SkeletonDCELHalfedge::X_monotone_curve * p = skeleton.new_segment( Segment_3(he->vertex()->point(), he->opposite()->vertex()->point()) );
        he->set_curve(p);
        he->opposite()->set_curve(p);
      }
    } else {
      assert(!he->has_null_curve());
    }
  }
  skeleton.assert_sane();

  finalized = true;
  DBG_FUNC_END(DBG_SKEL);
}

void
KineticTriangulation::initialize(
  const BasicInput& input,
  WavefrontEdgeList* p_wavefront_edges,
  int p_restrict_component
) {
  DBG_FUNC_BEGIN(DBG_KT_SETUP);

  assert(!initialized);
  wavefront_edges = p_wavefront_edges;
  restrict_component_ = p_restrict_component;

  /* set up basic triangulation */
  BasicTriangulation ct;
  ct.initialize(input);
  DBG(DBG_KT_SETUP) << "Input has " << ct.max_component() + 1 << " component(s): 0 .. " << ct.max_component();
  DBG(DBG_KT_SETUP) << "Requested straight skeleton of component " << restrict_component_;
  if (ct.max_component() < restrict_component_) {
    LOG(ERROR) << "Requested straight skeleton of component " << restrict_component_ << " but max component index is " << ct.max_component() << ".";
    exit(1);
  };

  FaceToTriangleIdxMap face_to_triangle_idx;
  TriangleOriginalVertexIndexList triangle_original_vertex_indices;

  /* initialze the basic triangulation data structure, setting up neighborhoods */
  const unsigned num_initial_triangles = get_num_initial_triangles(ct);
  DBG(DBG_KT_SETUP) << "  Have " << num_initial_triangles << " initial kinetic triangles.";
  initialize_tds(input, ct, num_initial_triangles,
    face_to_triangle_idx, triangle_original_vertex_indices);
  create_supporting_lines(input, ct, num_initial_triangles,
    face_to_triangle_idx, triangle_original_vertex_indices);
  create_kinetic_vertices(input, ct,
    triangle_original_vertex_indices);
  /* until here, triangle_original_vertex_indices is consistent.
   * create_bevels is the first that may flip things around.
   */
  create_bevels(input, ct,
    triangle_original_vertex_indices);
  store_initial_wavefront_vertices();

  tidx_in_check_refinement.resize(triangles.size(), false);
  refine_triangulation_initial();

  assert_valid(-1, CORE_ZERO);
  initialized = true;

  DBG_FUNC_END(DBG_KT_SETUP);
}

void
KineticTriangulation::
assert_valid() const {
  for (const auto & t: triangles) {
    if (t.is_dead()) continue;
    t.assert_valid();
  }
  assert(wavefront_edges);
  for (const auto & e: *wavefront_edges) {
    if (e.is_dead()) continue;
    assert(e.incident_triangle());
    assert(e.incident_triangle()->has_wavefront(&e));
    assert(! e.incident_triangle()->is_dead());
    // Remaining combinatorics is checked by triangle's assert_valid
  }
}

void
KineticTriangulation::
assert_valid(int current_component, const NT& time) const {
#if defined (DEBUG_EXPENSIVE_PREDICATES) && DEBUG_EXPENSIVE_PREDICATES >= 1
  DBG_FUNC_BEGIN(DBG_KT_EVENT2);
  assert_valid();
  for (const auto & t: triangles) {
    if (restrict_component_ >= 0) {
      /* We only do SK in one component */
      if (t.component != restrict_component_) continue;
    } else {
      /* We do SK in all components, but the time in the others will be earlier/later so triangles will not be right */
      if (t.component != current_component) continue;
    }
    if (t.is_dead()) continue;
    if (t.unbounded()) {
      // recall that unbounded triangles witness that the vertex ccw of the infinite
      // vertex is on the boundary of the CH.
      DBG(DBG_KT_EVENT2) << "t" << &t;
      unsigned idx = t.infinite_vertex_idx();
      const KineticTriangle* const n = t.neighbor(cw(idx));
      unsigned nidx = n->infinite_vertex_idx();

      const WavefrontVertex * const u = t.vertex(cw (idx));
      const WavefrontVertex * const v = t.vertex(ccw(idx));
      const WavefrontVertex * const V = n->vertex(cw (nidx));
      const WavefrontVertex * const w = n->vertex(ccw(nidx));
      assert(v==V);
      assert(CGAL::orientation(u->p_at(time),
                               v->p_at(time),
                               w->p_at(time)) != CGAL::RIGHT_TURN);
    } else {
      DBG(DBG_KT_EVENT2) << "t" << &t;
      const Point_2 a(t.vertex(0)->p_at(time));
      const Point_2 b(t.vertex(1)->p_at(time));
      const Point_2 c(t.vertex(2)->p_at(time));
      const NT det = compute_determinant(
          a.x(), a.y(),
          b.x(), b.y(),
          c.x(), c.y());
      assert_sign(det);
      if ((CGAL::orientation(a, b, c) == CGAL::RIGHT_TURN) != (det < 0)) {
        LOG(ERROR) << "CGAL is confused about orientation of triangle " << &t << ": determinant is " << CGAL::to_double(det) << "  but CGAL thinks orientation is " << CGAL::orientation(a, b, c);
        exit(EXIT_CGAL_ORIENTATION_MISMATCH);
      }
      assert(CGAL::orientation(a, b, c) != CGAL::RIGHT_TURN);
    }
  }
  for (const auto & v: vertices) {
    v.assert_valid();
  }
  DBG_FUNC_END(DBG_KT_EVENT2);
#else
  assert_valid();
  for (const auto & v: vertices) {
    v.assert_valid();
  }
#endif
}

void
KineticTriangulation::
move_constraints_to_neighbor(KineticTriangle& t, int idx) {
  CGAL_precondition(! (t.is_constrained(cw(idx)) && t.is_constrained(ccw(idx))) );

  for (unsigned i=1; i<=2; ++i) {
    if (t.is_constrained(mod3(idx+i))) {
      auto n  = t.neighbor(mod3(idx+3-i));
      int nidx = n->index(&t);
      CGAL_assertion(! n->is_constrained(nidx) );

      n->move_constraint_from(nidx, t, mod3(idx+i));
    }
  }
}

void
KineticTriangulation::
modified(KineticTriangle* t, bool front) {
  put_on_check_refinement(t, front);
  // In the initial refinement calls, we will
  // not have a queue yet.
  if (queue) {
    queue->needs_update(t);
  }

  if (t->unbounded()) {
    /* t's neighbor also witnesses the common vertex remaining in the wavefront.
     * let them know things might have changed.
     */
    unsigned idx = t->infinite_vertex_idx();
    KineticTriangle* n = t->neighbor(ccw(idx));
    assert(n);
    if (! n->is_dying()) {
      n->invalidate_collapse_spec();
      if (queue) {
        queue->needs_update(n);
      }
    }
  }
}

/** actually performs the flip, assert the triangulation is consistent before. */
void
KineticTriangulation::
do_raw_flip(KineticTriangle* t, unsigned idx, const NT& time, bool allow_collinear) {
  assert(t);
  assert(!t->is_constrained(idx));
  const KineticTriangle* n = t->neighbor(idx);
  int nidx = n->index(t);

  const WavefrontVertex * const v = t->vertex(idx);
  const WavefrontVertex * const v1 = t->vertex(ccw(idx));
  const WavefrontVertex * const v2 = t->vertex(cw (idx));
  const WavefrontVertex * const o = n->vertex(nidx);
  assert(v1 == n->vertex(cw (nidx)));
  assert(v2 == n->vertex(ccw(nidx)));

  /* We may also call this for two unbounded triangles.  However, right now we
   * only call this in one very specific way, so v1 is always the infinite and
   * v1 the finite one, and v, v1, and o are collinear on the CH boundary right
   * now.
   */
  assert(!v->is_infinite);
  assert(!v2->is_infinite);

  if (v1->is_infinite) {
    const Point_2 pos_v = v->p_at(time);
    const Point_2 pos_v2 = v2->p_at(time);
    const Point_2 pos_o = o->p_at(time);
    assert_expensive(CGAL::orientation(pos_v, pos_o, pos_v2) == CGAL::COLLINEAR);
  } else {
    const Point_2 pos_v = v->p_at(time);
    const Point_2 pos_v1 = v1->p_at(time);
    const Point_2 pos_v2 = v2->p_at(time);
    assert_expensive(CGAL::orientation(pos_v1, pos_v2, pos_v) != CGAL::RIGHT_TURN); // v may be on line(v1,v2)

    if (o->is_infinite) {
      /* Flipping to an unbounded triangle. */
      // nothing to do here.
    } else {
      const Point_2 pos_o = o->p_at(time);


      DBG(DBG_KT_EVENT2) << " o(v,o,v1):  " << CGAL::orientation(pos_v, pos_o, pos_v1);
      DBG(DBG_KT_EVENT2) << " o(v,o,v2):  " << CGAL::orientation(pos_v, pos_o, pos_v2);
      DBG(DBG_KT_EVENT2) << " o(v1,v2,o):  " << CGAL::orientation(pos_v1, pos_v2, pos_o);

      if (allow_collinear || true) {
        assert(CGAL::orientation(pos_v, pos_o, pos_v1) != CGAL::LEFT_TURN);
        assert(CGAL::orientation(pos_v, pos_o, pos_v2) != CGAL::RIGHT_TURN);
      } else {
        assert(CGAL::orientation(pos_v, pos_o, pos_v1) == CGAL::RIGHT_TURN);
        assert(CGAL::orientation(pos_v, pos_o, pos_v2) == CGAL::LEFT_TURN);
      }
      assert(CGAL::orientation(pos_v1, pos_v2, pos_o) != CGAL::LEFT_TURN); // The target triangle may be collinear even before.
    }
  }

  // not strictly necessary for flipping purpuses, but we probably
  // should not end up here if this doesn't hold:
  assert(!t->is_constrained(cw(idx)) || !t->is_constrained(ccw(idx)) || allow_collinear);
  t->do_raw_flip(idx);
}

/** perform a flip, marking t and its neighbor as modified. */
void
KineticTriangulation::
do_flip(KineticTriangle* t, unsigned idx, const NT& time, bool allow_collinear) {
  assert(t);
  assert(!t->is_constrained(idx));
  KineticTriangle* n = t->neighbor(idx);
  do_raw_flip(t, idx, time, allow_collinear);
  modified(t, true);
  modified(n);
}

/** process a flip event, checking all the assertions hold.  Calls do_flip(). */
void
KineticTriangulation::
do_flip_event(const NT& time, KineticTriangle& t, int edge_idx) {
  DBG_FUNC_BEGIN(DBG_KT_EVENT);
  DBG(DBG_KT_EVENT) << &t << "; flip edge " << edge_idx;

  assert(0 <= edge_idx && edge_idx < 3);

  assert(!t.wavefront(edge_idx));
  assert(!t.is_constrained(edge_idx));

  WavefrontVertex* v[3] = { t.vertices[    edge_idx ],
                            t.vertices[ccw(edge_idx)],
                            t.vertices[cw (edge_idx)] };
  const Point_2 p[3] = { v[0]->p_at(time),
                         v[1]->p_at(time),
                         v[2]->p_at(time) };
  const NT squared_lengths[3] = { CGAL::squared_distance(p[1],p[2]),
                                  CGAL::squared_distance(p[2],p[0]),
                                  CGAL::squared_distance(p[0],p[1]) };
  assert(squared_lengths[0] > squared_lengths[1]);
  assert(squared_lengths[0] > squared_lengths[2]);
  assert(squared_lengths[1] != CORE_ZERO);
  assert(squared_lengths[2] != CORE_ZERO);

  do_flip(&t, edge_idx, time);
  DBG_FUNC_END(DBG_KT_EVENT);
}

#ifdef REFINE_TRIANGULATION
void
KineticTriangulation::
refine_triangulation(KineticTriangle* t, const NT& time) {
  DBG_FUNC_BEGIN(DBG_KT_EVENT);
  DBG(DBG_KT_EVENT) << t;

  assert(t);
  t->assert_valid();

  if (!t->unbounded()) {
    const WavefrontVertex * const v0 = t->vertex(0);
    const WavefrontVertex * const v1 = t->vertex(1);
    const WavefrontVertex * const v2 = t->vertex(2);

    int num_reflex = (!v0->is_convex_or_straight()) +
                     (!v1->is_convex_or_straight()) +
                     (!v2->is_convex_or_straight());
    switch (num_reflex) {
      case 0:
        break;
      case 1: {
          int reflex_idx = (!v0->is_convex_or_straight()) ? 0 :
                           (!v1->is_convex_or_straight()) ? 1 :
                                                            2;
          assert(t->vertex(cw (reflex_idx))->is_convex_or_straight());
          assert(t->vertex(ccw(reflex_idx))->is_convex_or_straight());

          if (t->is_constrained(reflex_idx)) break;
          KineticTriangle* n = t->neighbor(reflex_idx);
          if (n->unbounded()) break;

          /* If on of the corners of the quadrilateral is actually straight (it won't be reflex), do not flip */
          unsigned idx_in_n = n->index(t);
          //if (t->is_constrained(ccw(reflex_idx)) && n->is_constrained(cw (idx_in_n)) && t->vertex(cw (reflex_idx))->is_reflex_or_straight()) break;
          //if (t->is_constrained(cw (reflex_idx)) && n->is_constrained(ccw(idx_in_n)) && t->vertex(ccw(reflex_idx))->is_reflex_or_straight()) break;

          const WavefrontVertex* v  = t->vertices[    reflex_idx];
          const WavefrontVertex* va = t->vertices[ccw(reflex_idx)];
          const WavefrontVertex* vb = t->vertices[cw (reflex_idx)];
          const WavefrontVertex* o  = n->vertices[idx_in_n];
          for (int i=0; i<=1; ++i) {
            assert(v ->wavefronts()[i]);
            assert(va->wavefronts()[i]);
            assert(vb->wavefronts()[i]);
            assert(o ->wavefronts()[i]);

            assert(v ->wavefronts()[i]->vertex(1-i) == v);
            assert(va->wavefronts()[i]->vertex(1-i) == va);
            assert(vb->wavefronts()[i]->vertex(1-i) == vb);
            assert(o ->wavefronts()[i]->vertex(1-i) == o);
          }
          /* If on of the corners of the quadrilateral is actually straight (it won't be reflex), do not flip */
          if (v->wavefronts()[1]->vertex(1) == va && va->is_reflex_or_straight()) break;
          if (v->wavefronts()[0]->vertex(0) == vb && vb->is_reflex_or_straight()) break;
          /* Either at v, or at the opposite vertex */
          // DBG(DBG_KT_REFINE) << "   o->wavefronts()[0]->vertex(0) " << o->wavefronts()[0]->vertex(0);
          // DBG(DBG_KT_REFINE) << "   o->wavefronts()[0]->vertex(1) " << o->wavefronts()[0]->vertex(1);
          // DBG(DBG_KT_REFINE) << "   o->wavefronts()[1]->vertex(0) " << o->wavefronts()[1]->vertex(0);
          // DBG(DBG_KT_REFINE) << "   o->wavefronts()[1]->vertex(1) " << o->wavefronts()[1]->vertex(1);
          if (o->wavefronts()[0]->vertex(0) == va && va->is_reflex_or_straight()) break;
          if (o->wavefronts()[1]->vertex(1) == vb && vb->is_reflex_or_straight()) break;

          DBG(DBG_KT_REFINE) << "  Flipping " << t->get_name() << " along " << reflex_idx;
          DBG(DBG_KT_REFINE) << "   t: " << t;
          DBG(DBG_KT_REFINE) << "   n: " << n;
          DBG(DBG_KT_REFINE) << "   v[" <<     reflex_idx  << "]: " << v  << " - " << CGAL_point(v ->p_at(time));
          DBG(DBG_KT_REFINE) << "   v[" << ccw(reflex_idx) << "]: " << va << " - " << CGAL_point(va->p_at(time));
          DBG(DBG_KT_REFINE) << "   v[" << cw (reflex_idx) << "]: " << vb << " - " << CGAL_point(vb->p_at(time));
          DBG(DBG_KT_REFINE) << "   n[o"                      "]: " << o  << " - " << CGAL_point(o ->p_at(time));

          do_flip(t, reflex_idx, time);
          break;
        };
      case 2:
        break;
      case 3:
        break;
      default:
        assert(false);
    }
  }

  DBG_FUNC_END(DBG_KT_EVENT);
}
#else
void
KineticTriangulation::
refine_triangulation(KineticTriangle* /* t */, const NT& /* time */) {
}
#endif

void
KineticTriangulation::
refine_triangulation_initial() {
  for (auto & t: triangles) {
    // Do _one_ refinement of each triangle.  If it actually does change
    // anything, then refine_triangulation() will have put the triangle
    // onto the check_refinement queue.  If not, no harm done.
    refine_triangulation(&t, CORE_ZERO);
  }
  process_check_refinement_queue(CORE_ZERO);
}

void
KineticTriangulation::
process_check_refinement_queue(const NT& time) {
  DBG_FUNC_BEGIN(DBG_KT_EVENT);

  while (!check_refinement.empty()) {
    KineticTriangle* t = check_refinement_pop();
    refine_triangulation(t, time);
  }

  DBG_FUNC_END(DBG_KT_EVENT);
}

KineticTriangle*
KineticTriangulation::
check_refinement_pop() {
  KineticTriangle* t = check_refinement.front();
  check_refinement.pop_front();
  assert(tidx_in_check_refinement[t->id]);
  tidx_in_check_refinement[t->id] = false;
  return t;
}

void
KineticTriangulation::
put_on_check_refinement(KineticTriangle* t, bool front) {
  assert(t);
  assert(tidx_in_check_refinement.size() > t->id);

  if (tidx_in_check_refinement[t->id]) return;

  tidx_in_check_refinement[t->id] = true;
  if (front) {
    check_refinement.push_front(t);
  } else {
    check_refinement.push_back(t);
  }
}

/** deal with spoke collapse
 *
 * The spoke at edge_idx collapsed, the incident vertices have been stopped
 * already.
 *
 * See if this triangle collapses entirely, or if its 3rd vertex (v) is elsewhere.
 *
 * If the triangle collapses entirely, we may have zero, one, or two constraints.
 * If it does not, we still may have zero or one constraint.
 * (Or two, if there is an infinitely fast vertex as v.)
 */
void
KineticTriangulation::
do_spoke_collapse_part2(KineticTriangle& t, unsigned edge_idx, const NT& time) {
  DBG_FUNC_BEGIN(DBG_KT_EVENT);
  DBG(DBG_KT_EVENT) << "t  " << &t;

  unsigned num_constraints = t.is_constrained(0) +  t.is_constrained(1) +  t.is_constrained(2);

  WavefrontVertex* v  = t.vertices[    edge_idx];
  WavefrontVertex* va = t.vertices[ccw(edge_idx)];
  WavefrontVertex* vb = t.vertices[cw (edge_idx)];
  assert(va->has_stopped());
  assert(vb->has_stopped());
  assert(va->pos_stop() == vb->pos_stop());
  assert(!v->has_stopped());
  assert(!t.neighbors[edge_idx]);

  const auto posa = va->pos_stop();

  if (!v->is_infinite && v->p_at(time) == posa) {
    t.set_dying();
    v->stop(time);
    /* triangle collapses completely */
    DBG(DBG_KT_EVENT) << "triangle collapses completely as spoke collapses; spoke is " << edge_idx;
    //LOG(WARNING) << __FILE__ << ":" << __LINE__ << " " << "untested code path.";

    int num_neighbors = 0;
    for (int i=1; i<=2; ++i) {
      int edge = mod3(edge_idx + i);
      DBG(DBG_KT_EVENT) << " at edge " << edge;
      if (t.neighbors[edge]) {
        DBG(DBG_KT_EVENT) << "  we have a neighbor";
        num_neighbors++;

        KineticTriangle *n = t.neighbors[edge];
        unsigned idx_in_n = n->index(&t);

        t.neighbors[edge] = NULL;
        n->neighbors[idx_in_n] = NULL;

        do_spoke_collapse_part2(*n, idx_in_n, time);
      } else {
        DBG(DBG_KT_EVENT) << "  we have no neighbor";
        //LOG(WARNING) << __FILE__ << ":" << __LINE__ << " " << "untested code path: DECL linking.";
        t.vertices[ccw(edge)]->set_next_vertex(1, t.vertices[cw(edge)], false);
        if (t.wavefront(edge)) {
          t.wavefront(edge)->set_dead();
        }
      }
    }
    queue->needs_dropping(&t);
  } else {
    bool call_constraint_collapse = true;
    if (v->infinite_speed != InfiniteSpeedType::NONE) {
      int i_1 = mod3(edge_idx + 1);
      int i_2 = mod3(edge_idx + 2);
      if (!t.neighbors[i_1] && !t.neighbors[i_2]) {
        DBG(DBG_KT_EVENT) << "triangle is fully constraint and has an infinitely fast vertex.";

        t.set_dying();
        v->stop(time, posa);
        for (int i=1; i<=2; ++i) {
          int edge = mod3(edge_idx + i);
          LOG(WARNING) << __FILE__ << ":" << __LINE__ << " " << "untested code path: DECL linking.";
          t.vertices[ccw(edge)]->set_next_vertex(1, t.vertices[cw(edge)], false);
          if (t.wavefront(edge)) {
            t.wavefront(edge)->set_dead();
          }
        }
        call_constraint_collapse = false;
        queue->needs_dropping(&t);
      }
    }

    if (call_constraint_collapse) {
      /* triangle does not collapse completely */
      DBG(DBG_KT_EVENT) << "triangle does not collapse completely";
      do_constraint_collapse_part2(t, edge_idx, time);
    }
  }

  DBG_FUNC_END(DBG_KT_EVENT);
}

void
KineticTriangulation::
handle_spoke_collapse_event(const Event& event) {
  DBG_FUNC_BEGIN(DBG_KT_EVENT);
  DBG(DBG_KT_EVENT) << event;

  assert(event.type() == CollapseType::SPOKE_COLLAPSE);
  KineticTriangle& t = triangles[event.t->id];
  const NT& time(event.time());
  int edge_idx = event.relevant_edge();

  assert(!t.is_constrained(edge_idx));

  WavefrontVertex* va = t.vertices[ccw(edge_idx)];
  WavefrontVertex* vb = t.vertices[cw (edge_idx)];
  va->stop(time);
  vb->stop(time);
  assert(va->pos_stop() == vb->pos_stop());

  KineticTriangle *n = t.neighbors[edge_idx];
  assert(n);
  unsigned idx_in_n = n->index(&t);

  t.neighbors[edge_idx] = NULL;
  n->neighbors[idx_in_n] = NULL;

  do_spoke_collapse_part2(t, edge_idx, time);
  do_spoke_collapse_part2(*n, idx_in_n, time);

  // update prev/next for the DCEL that is the wavefront vertices
  /* actually, nothing to do here, the two do_spoke_collapse_part2 calls did everything. */
  //LOG(WARNING) << __FILE__ << ":" << __LINE__ << " " << "untested code path: DECL linking.";

  assert_valid(t.component, time);

  DBG_FUNC_END(DBG_KT_EVENT);
}

void
KineticTriangulation::
handle_triangle_collapse_event(const Event& event) {
  DBG_FUNC_BEGIN(DBG_KT_EVENT);
  DBG(DBG_KT_EVENT) << event;

  assert(event.type() == CollapseType::TRIANGLE_COLLAPSE);
  KineticTriangle& t = triangles[event.t->id];
  const NT& time(event.time());

  unsigned num_constraints = t.is_constrained(0) +  t.is_constrained(1) +  t.is_constrained(2);
  DBG(DBG_KT_EVENT) << "have " << num_constraints << " constraints";

  t.set_dying();

  for (unsigned i=0; i<3; ++i) {
    t.vertices[i]->stop(time);
    DBG(DBG_KT_EVENT) << "v[" << i << "]: " << t.vertices[i];
  }
  for (unsigned i=1; i<3; ++i) {
    assert(t.vertices[0]->p_at(time) == t.vertices[i]->p_at(time));
    assert(t.vertices[0]->pos_stop() == t.vertices[i]->pos_stop());
  }
  for (unsigned i=0; i<3; ++i) {
    if (t.is_constrained(i)) {
      assert(t.wavefront(i) == t.vertex(ccw(i))->incident_wavefront_edge(1));
      assert(t.wavefront(i) == t.vertex(cw (i))->incident_wavefront_edge(0));

      t.wavefront(i)->set_dead();

      // update prev/next for the DCEL that is the wavefront vertices
      t.vertices[cw(i)]->set_next_vertex(0, t.vertices[ccw(i)], false);
    } else {
      // from the other triangle's point of view, a spoke collapsed.  deal with that.
      KineticTriangle *n = t.neighbors[i];
      assert(n);
      unsigned idx_in_n = n->index(&t);

      t.neighbors[i] = NULL;
      n->neighbors[idx_in_n] = NULL;

      do_spoke_collapse_part2(*n, idx_in_n, time);
    }
  }
  queue->needs_dropping(&t);

  assert_valid(t.component, time);

  DBG_FUNC_END(DBG_KT_EVENT);
}

/** Handle the 2nd part of a collapse of one constraint from a constraint event.
 *
 * After the old kinetic vertices are stopped, this creates a new kv, and updates
 * all incident triangles.
 *
 * We may also call this when a spoke collapses, in which case t.wavefront(edge_idx)
 * is NULL, but it will also not have a neighbor there anymore.
 */
void
KineticTriangulation::
do_constraint_collapse_part2(KineticTriangle& t, unsigned edge_idx, const NT& time) {
  DBG_FUNC_BEGIN(DBG_KT_EVENT);

  WavefrontVertex* va = t.vertices[ccw(edge_idx)];
  WavefrontVertex* vb = t.vertices[cw (edge_idx)];
  assert(va->has_stopped());
  assert(vb->has_stopped());
  assert_expensive_eq_ptr(va->pos_stop(), vb->pos_stop());

  auto pos = va->pos_stop();

  t.set_dying();
  move_constraints_to_neighbor(t, edge_idx);

  if (t.wavefront(edge_idx)) {
    /* constraint collapsed */
    assert(t.wavefront(edge_idx) == va->incident_wavefront_edge(1));
    assert(t.wavefront(edge_idx) == vb->incident_wavefront_edge(0));
  } else {
    /* spoke collapsed */
    assert(t.neighbor(edge_idx) == NULL);
  }
  WavefrontEdge const * const ea  = va->incident_wavefront_edge(0);
  WavefrontEdge const * const eb  = vb->incident_wavefront_edge(1);

  KineticTriangle * na = t.neighbors[cw (edge_idx)];
  KineticTriangle * nb = t.neighbors[ccw(edge_idx)];

  WavefrontVertex * const v = vertices.make_vertex(pos, time, ea, eb);
  DBG(DBG_KT) << " va: " << va;
  DBG(DBG_KT) << " vb: " << vb;
  va->set_next_vertex(0, v);
  vb->set_next_vertex(1, v);

  {
    DBG(DBG_KT_EVENT) << "updating vertex in affected triangles";
    auto end = incident_faces_end();
    auto i = incident_faces_iterator(&t, ccw(edge_idx));
    DEBUG_STMT(bool first = true);
    DBG(DBG_KT_EVENT) << " ccw:";
    for (--i; i != end; --i) {
      assert(!first || na == &*i); DEBUG_STMT(first = false);
      (*i).set_vertex(i.v_in_t_idx(), v);
      modified(&*i);
    };

    i = incident_faces_iterator(&t, cw(edge_idx));
    DEBUG_STMT(first = true);
    DBG(DBG_KT_EVENT) << " cw:";
    for (++i; i != end; ++i) {
      assert(!first || nb == &*i); DEBUG_STMT(first = false);
      (*i).set_vertex(i.v_in_t_idx(), v);
      modified(&*i);
    }
  };

  DBG(DBG_KT_EVENT) << " cw:";
  if (na) {
    auto idx = na->index(&t);
    assert(na->index(v) == ccw(idx));
    na->set_neighbor(idx, nb);
  }
  if (nb) {
    auto idx = nb->index(&t);
    assert(nb->index(v) == cw(idx));
    nb->set_neighbor(idx, na);
  }

  if (t.wavefront(edge_idx)) {
    /* if it was a constraint collapse rather than a spoke collapse */
    t.wavefront(edge_idx)->set_dead();
  }
  queue->needs_dropping(&t);

  DBG_FUNC_END(DBG_KT_EVENT);
}

void
KineticTriangulation::
handle_constraint_event(const Event& event) {
  DBG_FUNC_BEGIN(DBG_KT_EVENT);
  DBG(DBG_KT_EVENT) << event;

  assert(event.type() == CollapseType::CONSTRAINT_COLLAPSE);
  KineticTriangle& t = triangles[event.t->id];
  const NT& time(event.time());
  int edge_idx = event.relevant_edge();

  assert(t.wavefront(edge_idx));
  assert(t.wavefront(edge_idx)->get_collapse(t.component, time, edge_idx) == event);

  assert(t.is_constrained(edge_idx));

  WavefrontVertex* va = t.vertices[ccw(edge_idx)];
  WavefrontVertex* vb = t.vertices[cw (edge_idx)];
  va->stop(time);
  vb->stop(time);
  // DBG(DBG_KT_EVENT) << "va: " << CGAL_point(va->p_at(time));
  // DBG(DBG_KT_EVENT) << "vb: " << CGAL_point(vb->p_at(time));
  assert(va->p_at(time) == vb->p_at(time));

  // update prev/next for the DCEL that is the wavefront vertices
  va->set_next_vertex(1, vb, false);

  do_constraint_collapse_part2(t, edge_idx, time);

  assert_valid(t.component, time);

  DBG_FUNC_END(DBG_KT_EVENT);
}

void
KineticTriangulation::
handle_face_with_infintely_fast_opposing_vertex(const Event& event) {
  DBG_FUNC_BEGIN(DBG_KT_EVENT);
  DBG(DBG_KT_EVENT) << event;

  assert(event.type() == CollapseType::FACE_HAS_INFINITELY_FAST_VERTEX_OPPOSING);
  KineticTriangle& tref = triangles[event.t->id];
  KineticTriangle* t = &tref;
  const NT& time(event.time());

  DBG(DBG_KT_EVENT2) << "t:  " << t;

  unsigned num_constraints = t->is_constrained(0) +  t->is_constrained(1) +  t->is_constrained(2);
  unsigned num_fast = (t->vertex(0)->infinite_speed != InfiniteSpeedType::NONE) +
                      (t->vertex(1)->infinite_speed != InfiniteSpeedType::NONE) +
                      (t->vertex(2)->infinite_speed != InfiniteSpeedType::NONE);
  assert(num_fast >= 1);
  assert(num_fast < 3);
  if (num_constraints == 3) {
    DBG(DBG_KT_EVENT2) << "infinitely fast triangle with 3 constraints.";
    t->set_dying();

    Point_2 p;
    bool first=true;
    for (unsigned i=0; i<3; ++i) {
      if (t->vertex(i)->infinite_speed != InfiniteSpeedType::NONE) continue;
      t->vertices[i]->stop(time);
      if (first) {
        p = t->vertices[i]->pos_stop();
        first = false;
      } else {
        assert(p == t->vertices[i]->pos_stop());
      }
    }
    assert(!first);
    for (unsigned i=0; i<3; ++i) {
      if (t->vertex(i)->infinite_speed == InfiniteSpeedType::NONE) continue;
      t->vertices[i]->stop(time, p);
    }

    for (unsigned i=0; i<3; ++i) {
      t->wavefront(i)->set_dead();
    }

    // update prev/next for the DCEL that is the wavefront vertices
    for (unsigned i=0; i<3; ++i) {
      t->vertices[i]->set_next_vertex(0, t->vertices[cw(i)], false);
    }
    //LOG(WARNING) << __FILE__ << ":" << __LINE__ << " " << "untested code path: DECL linking.";

    queue->needs_dropping(t);
  } else {
    DBG(DBG_KT_EVENT2) << "infinitely fast triangle with fewer than 3 constraints.";
    assert(num_fast <= 2);
    unsigned t_fast_idx = t->infinite_speed_opposing_vertex_idx();
    WavefrontVertex* v = t->vertices[t_fast_idx];
    DBG(DBG_KT_EVENT2) << "infinitely fast vertex: " << t_fast_idx;

    AroundVertexIterator faces_it = incident_faces_iterator(t, t_fast_idx);
    AroundVertexIterator most_cw_triangle = faces_it.most_cw();

    DBG(DBG_KT_EVENT2) << "cw most triangle: " << most_cw_triangle;

    /* Flip away any spoke at the infinitely fast vertex. */
    /* ================================================== */
    DBG(DBG_KT_EVENT2) << "flipping all spokes away from " << most_cw_triangle.t()->vertex( most_cw_triangle.v_in_t_idx() );
    std::vector<KineticTriangle*> mark_modified;
    while (1) {
      unsigned nidx = ccw(most_cw_triangle.v_in_t_idx());
      if (most_cw_triangle.t()->is_constrained(nidx)) {
        break;
      }
      KineticTriangle *n = most_cw_triangle.t()->neighbor( nidx );
      DBG(DBG_KT_EVENT2) << "- flipping: " << most_cw_triangle.t();
      DBG(DBG_KT_EVENT2) << "  towards:  " << n;
      do_raw_flip(most_cw_triangle.t(), nidx, time, true);
      mark_modified.push_back(n);
    }
    DBG(DBG_KT_EVENT2) << "flipping done; " << most_cw_triangle;

    t = most_cw_triangle.t();
    DBG(DBG_KT_EVENT2) << "most cw triangle is: " << t;
    unsigned vidx_in_tc = most_cw_triangle.v_in_t_idx();

    assert(t->vertex(vidx_in_tc) == v);
    assert(t->is_constrained(cw (vidx_in_tc)));
    assert(t->is_constrained(ccw(vidx_in_tc)));

    /* Figure out which edge to retire */
    /* =============================== */
    WavefrontVertex* v_cw   = t->vertices[cw (vidx_in_tc)];
    WavefrontVertex* v_ccw  = t->vertices[ccw(vidx_in_tc)];

    WavefrontEdge *l = t->wavefront(ccw(vidx_in_tc));
    WavefrontEdge *r = t->wavefront(cw (vidx_in_tc));

    DBG(DBG_KT_EVENT) << "l is " << *l;
    DBG(DBG_KT_EVENT) << "r is " << *r;

    DBG(DBG_KT_EVENT) << "v     is " << v;
    DBG(DBG_KT_EVENT) << "v_cw  is " << v_cw;
    DBG(DBG_KT_EVENT) << "v_ccw is " << v_ccw;

    assert((v_cw->infinite_speed == InfiniteSpeedType::NONE) ||
           (v_ccw->infinite_speed == InfiniteSpeedType::NONE));

    unsigned collapse;
    WavefrontVertex* o = NULL;
    bool spoke_collapse = false;
    /* collapse the shorter edge, or the edge to the non-fast vertex (i.e, the one opposite of the fast). */
    if (v_cw->infinite_speed != InfiniteSpeedType::NONE) {
      collapse = cw(vidx_in_tc);
      o = v_ccw;
      DBG(DBG_KT_EVENT) << "v_cw has infinite speed";
    } else if (v_ccw->infinite_speed != InfiniteSpeedType::NONE) {
      collapse = ccw(vidx_in_tc);
      o = v_cw;
      DBG(DBG_KT_EVENT) << "v_ccw has infinite speed";
    } else {
      const auto pos = v->p_at(time);
      const auto poscw  = v_cw ->p_at(time);
      const auto posccw = v_ccw->p_at(time);
      const NT sq_length_v_to_vcw  = CGAL::squared_distance(pos, poscw );
      const NT sq_length_v_to_vccw = CGAL::squared_distance(pos, posccw);
      if (sq_length_v_to_vcw < sq_length_v_to_vccw) {
        collapse = ccw(vidx_in_tc);
        o = v_cw;
        DBG(DBG_KT_EVENT) << "sq_length_v_to_vcw < sq_length_v_to_vccw";
      } else if (sq_length_v_to_vcw > sq_length_v_to_vccw) {
        collapse = cw(vidx_in_tc);
        o = v_ccw;
        DBG(DBG_KT_EVENT) << "sq_length_v_to_vcw > sq_length_v_to_vccw";
      } else {
        DBG(DBG_KT_EVENT) << "sq_length_v_to_vcw == sq_length_v_to_vccw";
        spoke_collapse = true;
      }
    }
    if (spoke_collapse) {
      DBG(DBG_KT_EVENT) << "both edges incident to the infinitely fast vertex have the same length";
      t->set_dying();

      v_cw->stop(time);
      v_ccw->stop(time);
      assert(v_cw->pos_stop() == v_ccw->pos_stop());

      v->stop(time, v_cw->pos_stop());
      assert(t->wavefront(cw (vidx_in_tc)));
      assert(t->wavefront(ccw(vidx_in_tc)));
      t->wavefront(cw (vidx_in_tc))->set_dead();
      t->wavefront(ccw(vidx_in_tc))->set_dead();

      KineticTriangle *n = t->neighbors[vidx_in_tc];
      assert(n);
      unsigned idx_in_n = n->index(t);

      t->neighbors[vidx_in_tc] = NULL;
      n->neighbors[idx_in_n] = NULL;

      // update prev/next for the DCEL that is the wavefront vertices
      v->set_next_vertex(0, v_cw , false);
      v->set_next_vertex(1, v_ccw, false);

      assert(!t->is_dead());
      do_spoke_collapse_part2(*n, idx_in_n, time);

      assert(!t->is_dead());
      queue->needs_dropping(t);
    } else {
      DBG(DBG_KT_EVENT) << "collapse " << collapse;
      DBG(DBG_KT_EVENT) << "v " << v;
      DBG(DBG_KT_EVENT) << "o " << o;
      assert(t->index(v) + t->index(o) + collapse == 3);

      o->stop(time);
      v->stop(time, o->pos_stop());

      // update prev/next for the DCEL that is the wavefront vertices
      assert(o == v_ccw || o == v_cw);
      v->set_next_vertex( o == v_ccw ? 1 : 0, o, false);
      //LOG(WARNING) << __FILE__ << ":" << __LINE__ << " " << "untested code path: DECL linking.";

      do_constraint_collapse_part2(*t, collapse, time);
    }
    for (KineticTriangle* tm : mark_modified) {
      if (!tm->is_dead()) {
        modified(tm);
      } else {
        DBG(DBG_KT_EVENT) << "Not marking " << tm << " as modified because it is dead already.";
        assert(spoke_collapse);
      }
    }
  }
  assert_valid(t->component, time);

  DBG_FUNC_END(DBG_KT_EVENT);
}


void
KineticTriangulation::
handle_face_with_infintely_fast_weighted_vertex(const Event& event) {
  DBG_FUNC_BEGIN(DBG_KT_EVENT);
  DBG(DBG_KT_EVENT) << event;

  assert(event.type() == CollapseType::FACE_HAS_INFINITELY_FAST_VERTEX_WEIGHTED);
  KineticTriangle& tref = triangles[event.t->id];
  KineticTriangle* t = &tref;
  const NT& time(event.time());
  int edge_idx = event.relevant_edge();

  DBG(DBG_KT_EVENT2) << "t:  " << t;

  assert((t->vertex(0)->infinite_speed == InfiniteSpeedType::WEIGHTED) ||
         (t->vertex(1)->infinite_speed == InfiniteSpeedType::WEIGHTED) ||
         (t->vertex(2)->infinite_speed == InfiniteSpeedType::WEIGHTED));

  /* Find vertex with the fastest incident edge,
   * This will then gobble up one incident slower edge. */
  assert(t->wavefront(edge_idx));
  assert(t->wavefronts[edge_idx]->vertex(0) == t->vertices[ccw(edge_idx)]);
  assert(t->wavefronts[edge_idx]->vertex(1) == t->vertices[cw (edge_idx)]);

  assert((t->vertices[ccw(edge_idx)]->infinite_speed == InfiniteSpeedType::WEIGHTED) ||
         (t->vertices[cw (edge_idx)]->infinite_speed == InfiniteSpeedType::WEIGHTED));

  const WavefrontEdge * const winning_edge = t->wavefront(edge_idx);

  WavefrontVertex *v_fast;
  KineticTriangle *most_cw_triangle;
  int idx_fast_in_most_cw_triangle;
  int winning_edge_idx_in_v;
  DBG(DBG_KT_EVENT2) << " edge_idx:  " << edge_idx;
  DBG(DBG_KT_EVENT2) << " t->vertices[    edge_idx ]:  " << t->vertices[    edge_idx ];
  DBG(DBG_KT_EVENT2) << " t->vertices[ccw(edge_idx)]:  " << t->vertices[ccw(edge_idx)];
  DBG(DBG_KT_EVENT2) << " t->vertices[cw (edge_idx)]:  " << t->vertices[cw (edge_idx)];

  /* If both vertices are of type InfiniteSpeedType::WEIGHTED, pick one. */
  if (t->vertices[ccw(edge_idx)]->infinite_speed == InfiniteSpeedType::WEIGHTED) {
    /* The left vertex of the edge is the one in question. */
    v_fast = t->vertices[ccw(edge_idx)];
    winning_edge_idx_in_v = 1;
    most_cw_triangle = t;
    idx_fast_in_most_cw_triangle = ccw(edge_idx);
  } else {
    /* The right vertex of the edge is the one in question,
     * find the triangle that is incident to the other edge */
    v_fast = t->vertices[cw(edge_idx)];
    winning_edge_idx_in_v = 0;
    assert(t->wavefronts[edge_idx]->vertex(1)->wavefronts()[0] == t->wavefront(edge_idx));

    most_cw_triangle = t->wavefronts[edge_idx]->vertex(1)->wavefronts()[1]->incident_triangle();
    idx_fast_in_most_cw_triangle = most_cw_triangle->index(v_fast);
  };

  if (t->vertices[edge_idx]->is_infinite) {
    DBG(DBG_KT_EVENT2) << "Unbounded triangle";
    unsigned nidx = ccw(idx_fast_in_most_cw_triangle);
    KineticTriangle *n = most_cw_triangle->neighbor( nidx);
    assert(n->unbounded());
    unsigned idx_in_n = n->index(most_cw_triangle);
    assert(n->vertices[cw(idx_in_n)]->is_infinite);

    DBG(DBG_KT_EVENT2) << "- flipping: " << most_cw_triangle;
    DBG(DBG_KT_EVENT2) << "  towards:  " << n;

    do_raw_flip(most_cw_triangle, nidx, time, true);
    modified(n);
  } else {
    DBG(DBG_KT_EVENT2) << "Bounded triangle";

    /* The triangle should not have collapsed yet. */
    /*
     * Actually it may have
    {
      DBG(DBG_KT_EVENT2) << "most_cw_triangle is " << most_cw_triangle;
      const Point_2 pos_v0 = most_cw_triangle->vertex(0)->p_at(time);
      const Point_2 pos_v1 = most_cw_triangle->vertex(1)->p_at(time);
      const Point_2 pos_v2 = most_cw_triangle->vertex(2)->p_at(time);
      assert(CGAL::orientation(pos_v0, pos_v1, pos_v2) == CGAL::LEFT_TURN);
    }
    */

    /* Flip away any spoke at the infinitely fast vertex. */
    DBG(DBG_KT_EVENT2) << "flipping all spokes away from " << v_fast;
    AroundVertexIterator flipping_triangle = incident_faces_iterator(most_cw_triangle, idx_fast_in_most_cw_triangle);
    unsigned nidx_in_most_cw_triangle = ccw(idx_fast_in_most_cw_triangle);
    while (1) {
      assert(most_cw_triangle->vertices[idx_fast_in_most_cw_triangle] == v_fast);
      if (most_cw_triangle->is_constrained(nidx_in_most_cw_triangle)) {
        break;
      }

      const Point_2 pos_v0 = flipping_triangle.t()->vertex( ccw(flipping_triangle.v_in_t_idx()) )->p_at(time);
      const Point_2 pos_v1 = flipping_triangle.t()->vertex( cw (flipping_triangle.v_in_t_idx()) )->p_at(time);
      if (flipping_triangle.t() != most_cw_triangle) {
        /* We already delayed flipping at least once.  Let's see if we can go back */
        unsigned nidx = cw(flipping_triangle.v_in_t_idx()); /* previous guy, the one cw */
        KineticTriangle *n = flipping_triangle.t()->neighbor( nidx );
        unsigned idx_in_n = n->index(flipping_triangle.t());
        assert(n->vertex( ccw (idx_in_n) ) == flipping_triangle.t()->vertex( ccw (flipping_triangle.v_in_t_idx()) ) );
        const Point_2 pos_v2 = n->vertex(idx_in_n)->p_at(time);

        if (CGAL::orientation(pos_v2, pos_v0, pos_v1) != CGAL::RIGHT_TURN) {
          DBG(DBG_KT_EVENT2) << "- We can go back, and do a flip: " << flipping_triangle.t();
          ++flipping_triangle;
          continue; /* We will flip in the next iteration. */
        } else {
          DBG(DBG_KT_EVENT2) << "- We cannot go back just yet";
        }
      };

      /* Check if we can flip to the neighbor ccw. */
      unsigned nidx = ccw(flipping_triangle.v_in_t_idx()); /* next guy, the one ccw */
      KineticTriangle *n = flipping_triangle.t()->neighbor( nidx );
      unsigned idx_in_n = n->index(flipping_triangle.t());

      assert(n->vertex( cw (idx_in_n) ) == flipping_triangle.t()->vertex( cw (flipping_triangle.v_in_t_idx()) ) );
      const Point_2 pos_v2 = n->vertex(idx_in_n)->p_at(time);
      if (CGAL::orientation(pos_v0, pos_v1, pos_v2) == CGAL::RIGHT_TURN) {
        /* No, not right now.  Try in the next ccw triangle. */
        DBG(DBG_KT_EVENT2) << "- not flipping right now: " << flipping_triangle.t();
        --flipping_triangle;
      } else {
        DBG(DBG_KT_EVENT2) << "- flipping: " << flipping_triangle.t();
        DBG(DBG_KT_EVENT2) << "  towards:  " << n;
        do_raw_flip(flipping_triangle.t(), nidx, time, true);
        modified(n);
      }
    }
    DBG(DBG_KT_EVENT2) << "flipping done; " << most_cw_triangle;
  }

  assert(winning_edge == v_fast->wavefronts()[winning_edge_idx_in_v]);
  const WavefrontEdge * const losing_edge = v_fast->wavefronts()[1-winning_edge_idx_in_v];
  assert(v_fast == losing_edge->vertex(winning_edge_idx_in_v));
  WavefrontVertex* o = losing_edge->vertex(1-winning_edge_idx_in_v);

  DBG(DBG_KT_EVENT) << "v_fast " << v_fast;
  DBG(DBG_KT_EVENT) << "o      " << o;
  DBG(DBG_KT_EVENT) << "most_cw_triangle:  " << most_cw_triangle;
  DBG(DBG_KT_EVENT) << "winning edge at v: " << winning_edge_idx_in_v;

  if (o->infinite_speed != InfiniteSpeedType::NONE) {
    assert(o->infinite_speed == InfiniteSpeedType::WEIGHTED);
    o->stop(time, o->pos_start);
  } else {
    o->stop(time);
  }
  v_fast->stop(time, o->pos_stop());

  // update prev/next for the DCEL that is the wavefront vertices
  v_fast->set_next_vertex(1-winning_edge_idx_in_v, o, false);
  LOG(WARNING) << __FILE__ << ":" << __LINE__ << " " << "untested code path: DECL linking.";

  do_constraint_collapse_part2(*most_cw_triangle, most_cw_triangle->index(losing_edge), time);

  DBG_FUNC_END(DBG_KT_EVENT);
}

void
KineticTriangulation::
handle_split_event(const Event& event) {
  DBG_FUNC_BEGIN(DBG_KT_EVENT);
  DBG(DBG_KT_EVENT) << event;

  assert(event.type() == CollapseType::SPLIT_OR_FLIP_REFINE);
  KineticTriangle& t = triangles[event.t->id];
  const NT& time(event.time());
  int edge_idx = event.relevant_edge();

  DBG(DBG_KT_EVENT2) << " t:  " << &t;
  WavefrontVertex* v  = t.vertices[    edge_idx ];

  WavefrontEdge       * const e   = t.wavefront(edge_idx);
  WavefrontEdge const * const eb  = v->incident_wavefront_edge(0);
  WavefrontEdge const * const ea  = v->incident_wavefront_edge(1);
  assert(ea->vertex(0) == v);
  assert(eb->vertex(1) == v);

  KineticTriangle * na = t.neighbors[cw (edge_idx)];
  KineticTriangle * nb = t.neighbors[ccw(edge_idx)];

  { /* So, we have a wavefront edge e and we have an opposite reflex vertex v.
     * Assert that the vertex can actually hit the edge, i.e., check if the
     * wavefront edges e0 and e1 incident at v point away from e when
     * starting at v.
     *
     * The edges we store with v are directed such that e0 points towards v
     * and e1 away from it.
     *
     * The orientation needed for an event thus is that e and e0 form a right
     * turn, and e and e1 form a left turn.
     *
     * If either are collinear, things can still happen.
     */

    const Vector_2 e0(t.vertices[edge_idx]->incident_wavefront_edge(0)->l()->l.to_vector());
    const Vector_2 e1(t.vertices[edge_idx]->incident_wavefront_edge(1)->l()->l.to_vector());
    const CGAL::Orientation o0(CGAL::orientation(e->l()->l.to_vector(), e0));
    const CGAL::Orientation o1(CGAL::orientation(e->l()->l.to_vector(), e1));

    assert(o0 != CGAL::LEFT_TURN);
    assert(o1 != CGAL::RIGHT_TURN);

    #if 0
    if (o0 == CGAL::COLLINEAR ||
        o1 == CGAL::COLLINEAR) {
      NOTIMPL_MSG << "cannot handle collinear split event check yet";
      abort();
    }
    #endif
  }


  const Point_2& pos = v->pos_stop();
  // Stop the vertex
  v->stop(time);


  // Split edge e into parts,
  auto new_edges = e->split(*wavefront_edges);
  WavefrontEdge * const nea = new_edges.first;
  WavefrontEdge * const neb = new_edges.second;

  // Create new wavefront vertices with the new edges
  WavefrontVertex * const nva = vertices.make_vertex(pos, time, nea, ea, true);
  WavefrontVertex * const nvb = vertices.make_vertex(pos, time, eb, neb, true);

  // And set these new vertices on the edges.
  nea->vertex(0)->set_incident_wavefront_edge(1, nea);
  nea->set_wavefrontedge_vertex(1, nva);
  neb->set_wavefrontedge_vertex(0, nvb);
  neb->vertex(1)->set_incident_wavefront_edge(0, neb);

  // And update prev/next for the DCEL that is the wavefront vertices
  v->set_next_vertex(0, nvb);
  v->set_next_vertex(1, nva);
  nva->link_tail_to_tail(nvb);

  t.set_dying();
  {
    auto end = incident_faces_end();
    auto i = incident_faces_iterator(&t, edge_idx);
    KineticTriangle *lasta = NULL;
    DBG(DBG_KT_EVENT2) << " split: updating vertex on a side:";
    for (++i; i != end; ++i) {
      DBG(DBG_KT_EVENT2) << " split:   updating vertex on a side in " << &*i;
      (*i).set_vertex(i.v_in_t_idx(), nva);
      modified(&*i);
      lasta = &*i;
    };
    assert(lasta);
    assert(lasta->wavefront(cw(lasta->index(nva))));
    assert(lasta->wavefront(cw(lasta->index(nva)))->vertex(0) == nva);

    i = incident_faces_iterator(&t, edge_idx);
    KineticTriangle *lastb = NULL;
    DBG(DBG_KT_EVENT2) << " split: updating vertex on b side: ";
    for (--i; i != end; --i) {
      DBG(DBG_KT_EVENT2) << " split:   updating vertex on b side in " << &*i;
      (*i).set_vertex(i.v_in_t_idx(), nvb);
      modified(&*i);
      lastb = &*i;
    }
    assert(lastb);
    assert(lastb->wavefront(ccw(lastb->index(nvb))));
    assert(lastb->wavefront(ccw(lastb->index(nvb)))->vertex(1) == nvb);

    assert(na->index(nva) == cw( na->index(&t) ));
    na->set_wavefront(na->index(&t) , nea);

    assert(nb->index(nvb) == ccw( nb->index(&t)) );
    nb->set_wavefront(nb->index(&t), neb);

    DBG(DBG_KT_EVENT2) << " nea:" << *nea;
    DBG(DBG_KT_EVENT2) << " neb:" << *neb;
    DBG(DBG_KT_EVENT2) << " ea: " << *ea;
    DBG(DBG_KT_EVENT2) << " eb: " << *eb;
    DBG(DBG_KT_EVENT2) << " na: " << na;
    DBG(DBG_KT_EVENT2) << " nb: " << nb;

    na->assert_valid();
    nb->assert_valid();
    lasta->assert_valid();
    lastb->assert_valid();
  }

  queue->needs_dropping(&t);
  assert_valid(t.component, time);

  DBG_FUNC_END(DBG_KT_EVENT);
}

/* handle a split event, or refine this as a flip event.
 *
 * A triangle with exactly one constraint, e, has the opposite vertex v moving
 * onto the supporting line of e.
 *
 * This can be a split event, in which case we handle it right here,
 * or it can be a flip event, in which case we pump it down the line
 * so we can first deal with other, real split events or higher-priority flip
 * events.
 */
void
KineticTriangulation::
handle_split_or_flip_refine_event(const Event& event) {
  DBG_FUNC_BEGIN(DBG_KT_EVENT);
  DBG(DBG_KT_EVENT) << event;

  assert(event.type() == CollapseType::SPLIT_OR_FLIP_REFINE);
  KineticTriangle& t = triangles[event.t->id];
  const NT& time(event.time());
  int edge_idx = event.relevant_edge();

  DBG(DBG_KT_EVENT2) << " t:  " << &t;
  assert(t.wavefront(edge_idx));
  assert(t.is_constrained(edge_idx));
  assert(!t.is_constrained(cw (edge_idx)));
  assert(!t.is_constrained(ccw(edge_idx)));

  WavefrontVertex* v  = t.vertices[    edge_idx ];
  WavefrontVertex* va = t.vertices[ccw(edge_idx)];
  WavefrontVertex* vb = t.vertices[cw (edge_idx)];
  assert(! v->has_stopped());

  const auto posa = va->p_at(time);
  const auto posb = vb->p_at(time);
  const auto pos = v->p_at(time);
  const Segment_2 s(posa, posb);
  assert_expensive(s.supporting_line().has_on(pos));

  /* 3 cases:
   * (1) the vertex v is outside of the segment s, on the supporting line,
   * (2) it's at one of the endpoints of s,
   * (1) or v is on the interior of the edge.
   */

  const NT sq_length_constraint = s.squared_length();
  const NT sq_length_v_to_va = CGAL::squared_distance(pos, posa);
  const NT sq_length_v_to_vb = CGAL::squared_distance(pos, posb);
  const NT* longest_spoke = NULL;

  // DBG(DBG_KT_EVENT) << " v->pos: " << CGAL_point(pos);
  // DBG(DBG_KT_EVENT) << " va->pos: " << CGAL_point(posa);
  // DBG(DBG_KT_EVENT) << " vb->pos: " << CGAL_point(posb);
  // DBG(DBG_KT_EVENT) << " sqlength s: " << CGAL::to_double(sq_length_constraint);
  // DBG(DBG_KT_EVENT) << " sqlength v-va: " << CGAL::to_double(sq_length_v_to_va);
  // DBG(DBG_KT_EVENT) << " sqlength v-vb: " << CGAL::to_double(sq_length_v_to_vb);
  if (! s.collinear_has_on(pos)) {
    // case 1
    VLOG(2) << "A potential split event is actually a flip event.  Maybe refinement should have prevented that?";
    DBG(DBG_KT_EVENT) << "Re-classifying as flip event as v is not on the constrained segment.";

    assert(KineticTriangle::edge_is_faster_than_vertex(*t.vertex(edge_idx), *t.wavefront(edge_idx)->l()) != CGAL::NEGATIVE);

    /** there are basically two types of flip events involving constrained triangles.
     * One is where the vertex is coming towards the supporting line of the constraint
     * edge e and is passing left or right of of e.  The other is where (the
     * supporting line of an edge e overtakes a vertex.
     *
     * They are generally handled identically, however the assertions are slightly different
     * ones as these cases differ in which of v's incident edges is relevant.
     */
    const Vector_2 e(t.wavefront(edge_idx)->l()->l.to_vector());
    const Vector_2 e0(t.vertex(edge_idx)->incident_wavefront_edge(0)->l()->l.to_vector());
    const Vector_2 e1(t.vertex(edge_idx)->incident_wavefront_edge(1)->l()->l.to_vector());
    const CGAL::Orientation o0(CGAL::orientation(e, e0));
    const CGAL::Orientation o1(CGAL::orientation(e, e1));
    unsigned flip_edge;

    /* Figure out which side of the constraint edge we're on. */
    if (sq_length_v_to_va > sq_length_v_to_vb) {
      DBG(DBG_KT_EVENT) << "(v, va) is the longest spoke, so vb moves over that.";
      longest_spoke = &sq_length_v_to_va;

      assert(sq_length_v_to_va > sq_length_constraint); /* Check that v to va is the longest spoke */
      assert(t.index(vb) == cw(edge_idx));
      assert(vb->is_reflex_or_straight());

      /* If we come from behind, we don't really care about the first of these things in the discunjtion,
       * if we face it head on, we don't care about the second.  hmm. */
      assert(o1 != CGAL::RIGHT_TURN || (v->is_reflex_or_straight() && o0 != CGAL::RIGHT_TURN));
      //if (o1 == CGAL::COLLINEAR || o0 == CGAL::COLLINEAR) {
      #if 0
      if (o1 == CGAL::COLLINEAR) {
        NOTIMPL_MSG << "cannot handle collinear flip event yet";
        abort();
      }
      #endif

      // do_flip_event(event.time(), t, cw(edge_idx));
      flip_edge = cw(edge_idx);
    } else /* (pos, posa) < (pos, posb) */ {
      DBG(DBG_KT_EVENT) << "(v, vb) is the longest spoke, so va moves over that.";
      longest_spoke = &sq_length_v_to_vb;
      assert(sq_length_v_to_va != sq_length_v_to_vb); /* they really shouldn't be able to be equal. */

      assert(sq_length_v_to_vb > sq_length_constraint); /* Check that v to vb is the longest spoke */
      assert(t.index(va) == ccw(edge_idx));
      assert(va->is_reflex_or_straight());

      /* If we come from behind, we don't really care about the first of these things in the discunjtion,
       * if we face it head on, we don't care about the second.  hmm. */
      assert(o0 != CGAL::LEFT_TURN || (v->is_reflex_or_straight() && o1 != CGAL::LEFT_TURN));
      //if (o0 == CGAL::COLLINEAR || o1 == CGAL::COLLINEAR) {
      #if 0
      if (o0 == CGAL::COLLINEAR) {
        NOTIMPL_MSG << "cannot handle collinear flip event yet";
        abort();
      }
      #endif

      // do_flip_event(event.time(), t, ccw(edge_idx));
      flip_edge = ccw(edge_idx);
    };
    const CollapseSpec& c = t.refine_collapse_spec( CollapseSpec(t.component, CollapseType::VERTEX_MOVES_OVER_SPOKE, time, flip_edge, *longest_spoke) );
    DBG(DBG_KT_EVENT) << " Refining to " << c;
    queue->needs_update(&t, true);
  } else if (pos == posa) {
    // case 2
    const CollapseSpec& c = t.refine_collapse_spec( CollapseSpec(t.component, CollapseType::SPOKE_COLLAPSE, time, cw (edge_idx)) );
    DBG(DBG_KT_EVENT) << " v is incident to va Refining to " << c;
    queue->needs_update(&t, true);
  } else if (pos == posb) {
    // case 2
    const CollapseSpec& c = t.refine_collapse_spec( CollapseSpec(t.component, CollapseType::SPOKE_COLLAPSE, time, ccw(edge_idx)) );
    DBG(DBG_KT_EVENT) << " v is incident to vb Refining to " << c;
    queue->needs_update(&t, true);
  } else {
    DBG(DBG_KT_EVENT) << "We have a real split event.";
    handle_split_event(event);
  }

  DBG_FUNC_END(DBG_KT_EVENT);
}

void
KineticTriangulation::
handle_vertex_moves_over_spoke_event(const Event& event) {
  DBG_FUNC_BEGIN(DBG_KT_EVENT);
  DBG(DBG_KT_EVENT) << event;

  assert(event.type() == CollapseType::VERTEX_MOVES_OVER_SPOKE);
  KineticTriangle& t = triangles[event.t->id];

  do_flip_event(event.time(), t, event.relevant_edge());

  DBG_FUNC_END(DBG_KT_EVENT);
}

void
KineticTriangulation::
handle_ccw_vertex_leaves_ch_event(const Event& event) {
  DBG_FUNC_BEGIN(DBG_KT_EVENT);
  DBG(DBG_KT_EVENT) << event;

  assert(event.type() == CollapseType::CCW_VERTEX_LEAVES_CH);
  KineticTriangle& t = triangles[event.t->id];
  int idx = event.relevant_edge(); /* finite edge idx == infinite vertex idx */

  assert((unsigned)idx == t.infinite_vertex_idx());
  do_flip(&t, cw(idx), event.time());

  DBG_FUNC_END(DBG_KT_EVENT);
}

void
KineticTriangulation::
handle_event(const Event& event) {
  DBG_FUNC_BEGIN(DBG_KT | DBG_KT_EVENT);
  DBG(DBG_KT_EVENT);
  DBG(DBG_KT_EVENT);
  DBG(DBG_KT_EVENT);
  DBG(DBG_KT | DBG_KT_EVENT) << event;
  assert(!finalized);

  const NT& time = event.time();

#ifdef NT_USE_DOUBLE
  static NT last_time = 0;
  static unsigned count = 0;
  if (time <= last_time) {
    ++count;
    if (count > 10000) {
      LOG(ERROR) << "In double loop at line " << __FILE__ << ":" << __LINE__;
      abort();
    };
  } else {
    count = 0;
    last_time = time;
  };
#endif

  assert(&triangles[event.t->id] == event.t);

  assert(check_refinement.empty());
  assert(CollapseSpec(event) == event.t->get_collapse(time));

  ++ event_type_counter[int(CollapseType::UNDEFINED)];
  ++ event_type_counter[int(event.type())];

  switch (event.type()) {
    case CollapseType::TRIANGLE_COLLAPSE:
      handle_triangle_collapse_event(event);
      break;
    case CollapseType::CONSTRAINT_COLLAPSE:
      handle_constraint_event(event);
      break;
    case CollapseType::FACE_HAS_INFINITELY_FAST_VERTEX_OPPOSING:
      handle_face_with_infintely_fast_opposing_vertex(event);
      break;
    case CollapseType::FACE_HAS_INFINITELY_FAST_VERTEX_WEIGHTED:
      handle_face_with_infintely_fast_weighted_vertex(event);
      break;
    case CollapseType::SPOKE_COLLAPSE:
      handle_spoke_collapse_event(event);
      break;
    case CollapseType::SPLIT_OR_FLIP_REFINE:
      handle_split_or_flip_refine_event(event);
      break;
    case CollapseType::VERTEX_MOVES_OVER_SPOKE:
      handle_vertex_moves_over_spoke_event(event);
      break;
    case CollapseType::CCW_VERTEX_LEAVES_CH:
      handle_ccw_vertex_leaves_ch_event(event);
      break;
    /*
    case CollapseType::GENERIC_FLIP_EVENT:
      assert(false);
      exit(1);
      break;
    */
    case CollapseType::INVALID_EVENT:
    case CollapseType::UNDEFINED:
    case CollapseType::NEVER:
      CANNOTHAPPEN_MSG << "Should not get event " << event << " to handle.";
      assert(false);
      exit(1);
      break;
    default:
      CANNOTHAPPEN_MSG << "Unexpected event " << event;
      assert(false);
      exit(1);
      break;
  };
  DBG(DBG_KT_REFINE) << event << " - done.  doing refinement";
  process_check_refinement_queue(time);

  DBG_FUNC_END(DBG_KT | DBG_KT_EVENT);
}

std::ostream& operator<<(std::ostream& os, const KineticTriangulation::AroundVertexIterator it) {
  os << "avi(v"
     << it.v_in_t_idx()
     << " @ "
     << it.t()
     << ")";
  return os;
}
