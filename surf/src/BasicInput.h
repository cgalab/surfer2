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

#include "BGLGraph.h"
#include "tools.h"
#include "cgaltools.h"

#include <map>
#include <set>
#include <utility>

class BasicVertex {
  public:
    const Point_2 p;
    const unsigned degree;
    const unsigned id;
    const unsigned reflex_beveling_add;

    BasicVertex(const Point_2& p_p, unsigned p_degree, unsigned p_id)
      : p(p_p)
      , degree(p_degree)
      , id(p_id)
      , reflex_beveling_add(0) // number of kinetic vertices to create additionally at reflex vertices.
                               // At degree one vertices, we always add at least one.
      {}
};

class BasicEdge {
  public:
    const unsigned u, v;
    const NT weight;

    BasicEdge(unsigned p_u, unsigned p_v, const NT &p_weight=1.0)
      : u(p_u)
      , v(p_v)
      , weight(p_weight)
      {
        assert(u < v);
      }
};

class BasicInput {
  using Vertex = BasicVertex;
  using Edge = BasicEdge;

  using VertexList = std::vector<Vertex>;
  using EdgeList = std::vector<Edge>;
  using VertexIdxPair = std::pair<unsigned,unsigned>;

  private:
    unsigned num_of_deg1_vertices = 0;
    VertexList vertices_;
    EdgeList edges_;
    std::map<VertexIdxPair, unsigned> edge_map;
    /* keep a list of instances of our number type for the different weights. */
    std::set<NT> weight_set;

    /** Add an input vertex to the vertexlist */
    inline void add_vertex(Vertex&& p) {
      vertices_.emplace_back(std::forward<Vertex>(p));
    }
    /** Add an input edge between vertices to the edgelist */
    inline void add_edge(unsigned u, unsigned v, const NT& weight=1.0) {
      sort_tuple(u,v);
      assert(u < vertices_.size());
      assert(v < vertices_.size());
      assert(u!=v);

      auto wsinsert_res = weight_set.insert(weight);
      edges_.emplace_back(Edge(u,v, *wsinsert_res.first));
      auto res = edge_map.emplace(std::pair<VertexIdxPair,unsigned>(VertexIdxPair(u,v), edges().size()-1));
      assert(res.second);
    }

    #ifndef NDEBUG
    void assert_valid() const;
    #else
    void assert_valid() const {};
    #endif
  public:
    const VertexList& vertices() const { return vertices_; };
    const EdgeList& edges() const { return edges_; };
    void add_graph(const BGLGraph& graph);
    unsigned get_num_of_deg1_vertices() const {
      return num_of_deg1_vertices;
    }
    unsigned get_total_degree() const {
      return edges().size() * 2;
    }
    unsigned get_num_extra_beveling_vertices() const {
      /* Includes the extra one vertex we'll need at a minimum for degree-1 vertices. */
      return num_of_deg1_vertices;
    }
    bool has_edge(unsigned u, unsigned v) const {
      sort_tuple(u,v);
      auto findres = edge_map.find(VertexIdxPair(u, v));
      return findres != edge_map.end();
    }
    const Edge& get_edge(unsigned u, unsigned v) const {
      sort_tuple(u,v);
      assert(has_edge(u,v));
      auto findres = edge_map.find(VertexIdxPair(u, v));
      assert(findres != edge_map.end());
      return edges_[findres->second];
    }

    Segment_2 get_segment(const Edge& e) const {
      return Segment_2(vertices_[e.u].p, vertices_[e.v].p);
    }
};
