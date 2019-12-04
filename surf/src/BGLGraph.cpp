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
#include "BGLGraph.h"

BGLGraph
BGLGraph::
create_from_graphml(std::istream &istream) {
  typedef BGLGraph::vertex_property_type VertexPropertyType;
  typedef BGLGraph::edge_property_type   EdgePropertyType;

  BGLGraph graph;
  graph.dp.property("vertex-coordinate-x", boost::get(&VertexPropertyType::x, graph));
  graph.dp.property("vertex-coordinate-y", boost::get(&VertexPropertyType::y, graph));
  graph.dp.property("edge-weight",          boost::get(&EdgePropertyType::weight, graph));
  graph.dp.property("edge-weight-additive", boost::get(&EdgePropertyType::weight_additive, graph));

  boost::read_graphml(istream, graph, graph.dp);
  return graph;
}


std::ostream&
operator<<(std::ostream& os, const BGLGraph& g) {
  typedef BGLGraph::vertex_descriptor VertexType;
  auto index_map = boost::get(boost::vertex_index, g);

  os << "vertices(g):" << std::endl;
  for (auto vp = boost::vertices(g); vp.first != vp.second; ++vp.first) {
    const VertexType v = *vp.first;
    os << index_map[v] <<  " " << g[v].x << " " << g[v].y << std::endl;
  }
  return os;
}
