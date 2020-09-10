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
#include "BasicInput.h"

BasicInputFromBGL::
BasicInputFromBGL(const BGLGraph& graph) {
  typedef BGLGraph::vertex_descriptor VertexType;
  typedef BGLGraph::edge_descriptor EdgeType;
  DEBUG_STMT(auto index_map = boost::get(boost::vertex_index, graph));

  for (auto vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
    const VertexType v = *vp.first;
    DBG(DBG_INPUT) << "x: " << graph[v].x << "; y: " << graph[v].y;
    Point_2 p(string_to_maybe_NT(graph[v].x), string_to_maybe_NT(graph[v].y));
    DBG(DBG_INPUT) << std::setprecision(16) << "x: " << CGAL::to_double(p.x()) << "; y: " << CGAL::to_double(p.y());
    unsigned degree = boost::degree(v, graph);
    add_vertex(Vertex(p, degree, num_vertices_()));
    assert(index_map[v] == num_vertices_()-1);
  }
  for (auto ep = boost::edges(graph); ep.first != ep.second; ++ep.first) {
    const EdgeType e = *ep.first;
    const NT weight((graph[e].weight == "") ? CORE_ONE : string_to_maybe_NT(graph[e].weight));
    unsigned s = source(e, graph);
    unsigned t = target(e, graph);
    if (s == t) {
      LOG(ERROR) << "Invalid input: source and target of edge " << num_edges_() << " are the same vertex (v" << t << ").";
      exit(EXIT_INVALID_INPUT);
    }
    add_edge(s, t, weight);
  }

  finalize();
}

#ifndef SURF_NDEBUG
void
BasicInput::assert_valid() const {
  unsigned* d = new unsigned[vertices_.size()]();
  unsigned deg1 = 0;

  assert(edge_map.size() == edges_.size());
  for (size_t i=0; i<edges_.size(); ++i) {
    const auto & e = edges_[i];
    DBG(DBG_INPUT) << i << ": " << e.u << " " << e.v;
    assert(e.u < vertices_.size());
    assert(e.v < vertices_.size());
    d[e.u]++;
    d[e.v]++;
    auto findres = edge_map.find(VertexIdxPair(e.u,e.v));
    assert(findres != edge_map.end());
    assert(findres->second == i);
  }
  for (size_t i=0; i<vertices_.size(); ++i) {
    const auto & v = vertices_[i];
    assert(d[i] == v.degree);
    if (d[i] == 1) deg1++;
  }
  assert(deg1 == num_of_deg1_vertices);
  delete[] d;
}
#endif
