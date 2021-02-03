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

