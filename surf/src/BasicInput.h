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

#include "BGLGraph.h"
#include "tools.h"
#include "cgaltools.h"

#include <map>
#include <set>
#include <utility>

class BasicVertex {
  public:
    DEBUG_DECL(
    const unsigned id;
    )
    const Point_2 p;
    const unsigned degree;
    const unsigned reflex_beveling_add;

    BasicVertex(const Point_2& p_p, unsigned p_degree, unsigned p_id)
      :
      #ifndef SURF_NDEBUG
        id(p_id),
      #endif
        p(p_p)
      , degree(p_degree)
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
  protected:
    using VertexIdxPair = std::pair<unsigned,unsigned>;

  public:
    using Vertex = BasicVertex;
    using Edge = BasicEdge;
    using VertexList = std::vector<Vertex>;
    using EdgeList = std::vector<Edge>;

  private:
    bool finalized = false;

    unsigned num_of_deg1_vertices = 0;
    VertexList vertices_;
    EdgeList edges_;
    std::map<VertexIdxPair, unsigned> edge_map;
    /* keep a list of instances of our number type for the different weights. */
    std::set<NT> weight_set;

  protected:
    /** Add an input vertex to the vertexlist */
    void add_vertex(Vertex&& p) {
      assert(!finalized);
      if (p.degree == 1) num_of_deg1_vertices++;
      vertices_.emplace_back(std::forward<Vertex>(p));
    }

    unsigned num_vertices_() const { return vertices_.size(); };
    unsigned num_edges_() const { return edges_.size(); };

    /** Add an input edge between vertices to the edgelist */
    void add_edge(unsigned u, unsigned v, const NT& weight=1.0) {
      assert(!finalized);
      sort_tuple(u,v);
      assert(u < vertices_.size());
      assert(v < vertices_.size());
      assert(u!=v);

      auto wsinsert_res = weight_set.insert(weight);
      edges_.emplace_back(Edge(u,v, *wsinsert_res.first));
      [[maybe_unused]] auto res = edge_map.emplace(std::pair<VertexIdxPair,unsigned>(VertexIdxPair(u,v), edges_.size()-1));
      assert(res.second);
    }

    /* Call once all input has been loaded */
    void finalize() {
      assert(!finalized);
      finalized = true;
      assert_valid();
    }

    bool is_finalized() const { return finalized; };

    #ifndef SURF_NDEBUG
    void assert_valid() const;
    #else
    void assert_valid() const {};
    #endif

  public:
    /* All these methods only work on finalized graphs */
    const VertexList& vertices() const { assert(finalized); return vertices_; };
    const EdgeList& edges() const { assert(finalized); return edges_; };

    unsigned get_num_of_deg1_vertices() const {
      assert(finalized);
      return num_of_deg1_vertices;
    }
    unsigned get_total_degree() const {
      assert(finalized);
      return edges().size() * 2;
    }
    unsigned get_num_extra_beveling_vertices() const {
      assert(finalized);
      /* Includes the extra one vertex we'll need at a minimum for degree-1 vertices. */
      return num_of_deg1_vertices;
    }
    bool has_edge(unsigned u, unsigned v) const {
      assert(finalized);
      sort_tuple(u,v);
      auto findres = edge_map.find(VertexIdxPair(u, v));
      return findres != edge_map.end();
    }
    const Edge& get_edge(unsigned u, unsigned v) const {
      assert(finalized);
      sort_tuple(u,v);
      assert(has_edge(u,v));
      auto findres = edge_map.find(VertexIdxPair(u, v));
      assert(findres != edge_map.end());
      return edges_[findres->second];
    }

    Segment_2 get_segment(const Edge& e) const {
      assert(finalized);
      return Segment_2(vertices_[e.u].p, vertices_[e.v].p);
    }
};

class BasicInputFromBGL : public BasicInput {
  public:
    BasicInputFromBGL(const BGLGraph& graph);
};
