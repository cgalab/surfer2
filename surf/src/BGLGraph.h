#pragma once

#include "surf.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <string>

struct BGLVertexPropertyType {
  std::string x, y;
};

struct BGLEdgePropertyType {
  std::string weight;
  std::string weight_additive;
};

class BGLGraph : public boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, BGLVertexPropertyType, BGLEdgePropertyType> {
  private:
    using Base = boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, BGLVertexPropertyType, BGLEdgePropertyType>;

    boost::dynamic_properties dp;

  public:
    typedef typename Base::vertex_property_type      vertex_property_type;
    typedef typename Base::vertex_descriptor         vertex_descriptor;

  public:
    BGLGraph()
      : Base()
      , dp(boost::ignore_other_properties)
      {};

    static BGLGraph create_from_graphml(std::istream &istream);

  friend std::ostream& operator<<(std::ostream& os, const BGLGraph& e);
};

