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
#include "gtest/gtest.h"

#include <sstream>


namespace{

const char * graphml_graph_st0000054 = R"GRAPHML(
<graphml xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://graphml.graphdrawing.org/xmlns" xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">
  <key attr.name="vertex-coordinate-x" attr.type="string" for="node" id="x"/>
  <key attr.name="vertex-coordinate-y" attr.type="string" for="node" id="y"/>
  <key attr.name="edge-weight" attr.type="string" for="edge" id="w"/>
  <key attr.name="edge-weight-additive" attr.type="string" for="edge" id="wa"/>
  <graph edgedefault="undirected">
    <node id="0">
      <data key="x">-1.0</data>
      <data key="y">-0.208333</data>
    </node>
    <node id="1">
      <data key="x">-0.875</data>
      <data key="y">-0.208333</data>
    </node>
    <node id="2">
      <data key="x">-0.791667</data>
      <data key="y">-0.208333</data>
    </node>
    <node id="3">
      <data key="x">-0.708333</data>
      <data key="y">-0.208333</data>
    </node>
    <node id="4">
      <data key="x">-0.583333</data>
      <data key="y">-0.208333</data>
    </node>
    <node id="5">
      <data key="x">-0.5</data>
      <data key="y">-0.208333</data>
    </node>
    <node id="6">
      <data key="x">-0.416667</data>
      <data key="y">-0.208333</data>
    </node>
    <node id="7">
      <data key="x">-0.291667</data>
      <data key="y">-0.208333</data>
    </node>
    <node id="8">
      <data key="x">-0.208333</data>
      <data key="y">-0.208333</data>
    </node>
    <node id="9">
      <data key="x">-0.125</data>
      <data key="y">-0.208333</data>
    </node>
    <node id="10">
      <data key="x">-0.0</data>
      <data key="y">-0.208333</data>
    </node>
    <node id="11">
      <data key="x">-0.0</data>
      <data key="y">-0.166667</data>
    </node>
    <node id="12">
      <data key="x">-0.0</data>
      <data key="y">-0.125</data>
    </node>
    <node id="13">
      <data key="x">-0.0</data>
      <data key="y">-0.083333</data>
    </node>
    <node id="14">
      <data key="x">-0.0</data>
      <data key="y">-0.041667</data>
    </node>
    <node id="15">
      <data key="x">-0.0</data>
      <data key="y">-0.0</data>
    </node>
    <node id="16">
      <data key="x">-0.125</data>
      <data key="y">-0.0</data>
    </node>
    <node id="17">
      <data key="x">-0.166667</data>
      <data key="y">-0.041667</data>
    </node>
    <node id="18">
      <data key="x">-0.125</data>
      <data key="y">-0.083333</data>
    </node>
    <node id="19">
      <data key="x">-0.166667</data>
      <data key="y">-0.125</data>
    </node>
    <node id="20">
      <data key="x">-0.125</data>
      <data key="y">-0.166667</data>
    </node>
    <node id="21">
      <data key="x">-0.208333</data>
      <data key="y">-0.166667</data>
    </node>
    <node id="22">
      <data key="x">-0.291667</data>
      <data key="y">-0.166667</data>
    </node>
    <node id="23">
      <data key="x">-0.25</data>
      <data key="y">-0.125</data>
    </node>
    <node id="24">
      <data key="x">-0.291667</data>
      <data key="y">-0.083333</data>
    </node>
    <node id="25">
      <data key="x">-0.25</data>
      <data key="y">-0.041667</data>
    </node>
    <node id="26">
      <data key="x">-0.291667</data>
      <data key="y">-0.0</data>
    </node>
    <node id="27">
      <data key="x">-0.416667</data>
      <data key="y">-0.0</data>
    </node>
    <node id="28">
      <data key="x">-0.375</data>
      <data key="y">-0.041667</data>
    </node>
    <node id="29">
      <data key="x">-0.416667</data>
      <data key="y">-0.083333</data>
    </node>
    <node id="30">
      <data key="x">-0.375</data>
      <data key="y">-0.125</data>
    </node>
    <node id="31">
      <data key="x">-0.416667</data>
      <data key="y">-0.166667</data>
    </node>
    <node id="32">
      <data key="x">-0.5</data>
      <data key="y">-0.166667</data>
    </node>
    <node id="33">
      <data key="x">-0.583333</data>
      <data key="y">-0.166667</data>
    </node>
    <node id="34">
      <data key="x">-0.625</data>
      <data key="y">-0.125</data>
    </node>
    <node id="35">
      <data key="x">-0.583333</data>
      <data key="y">-0.083333</data>
    </node>
    <node id="36">
      <data key="x">-0.625</data>
      <data key="y">-0.041667</data>
    </node>
    <node id="37">
      <data key="x">-0.583333</data>
      <data key="y">-0.0</data>
    </node>
    <node id="38">
      <data key="x">-0.708333</data>
      <data key="y">-0.0</data>
    </node>
    <node id="39">
      <data key="x">-0.75</data>
      <data key="y">-0.041667</data>
    </node>
    <node id="40">
      <data key="x">-0.708333</data>
      <data key="y">-0.083333</data>
    </node>
    <node id="41">
      <data key="x">-0.75</data>
      <data key="y">-0.125</data>
    </node>
    <node id="42">
      <data key="x">-0.708333</data>
      <data key="y">-0.166667</data>
    </node>
    <node id="43">
      <data key="x">-0.791667</data>
      <data key="y">-0.166667</data>
    </node>
    <node id="44">
      <data key="x">-0.875</data>
      <data key="y">-0.166667</data>
    </node>
    <node id="45">
      <data key="x">-0.833333</data>
      <data key="y">-0.125</data>
    </node>
    <node id="46">
      <data key="x">-0.875</data>
      <data key="y">-0.083333</data>
    </node>
    <node id="47">
      <data key="x">-0.833333</data>
      <data key="y">-0.041667</data>
    </node>
    <node id="48">
      <data key="x">-0.875</data>
      <data key="y">-0.0</data>
    </node>
    <node id="49">
      <data key="x">-1.0</data>
      <data key="y">-0.0</data>
    </node>
    <node id="50">
      <data key="x">-1.0</data>
      <data key="y">-0.041667</data>
    </node>
    <node id="51">
      <data key="x">-1.0</data>
      <data key="y">-0.083333</data>
    </node>
    <node id="52">
      <data key="x">-1.0</data>
      <data key="y">-0.125</data>
    </node>
    <node id="53">
      <data key="x">-1.0</data>
      <data key="y">-0.166667</data>
    </node>
    <edge source="10" target="11"/>
    <edge source="22" target="23"/>
    <edge source="42" target="43"/>
    <edge source="0" target="53"/>
    <edge source="5" target="6"/>
    <edge source="28" target="29"/>
    <edge source="36" target="37"/>
    <edge source="8" target="9"/>
    <edge source="15" target="16"/>
    <edge source="40" target="41"/>
    <edge source="18" target="19"/>
    <edge source="30" target="31"/>
    <edge source="34" target="35"/>
    <edge source="46" target="47"/>
    <edge source="31" target="32"/>
    <edge source="1" target="2"/>
    <edge source="16" target="17"/>
    <edge source="32" target="33"/>
    <edge source="49" target="50"/>
    <edge source="6" target="7"/>
    <edge source="51" target="52"/>
    <edge source="12" target="13"/>
    <edge source="20" target="21"/>
    <edge source="44" target="45"/>
    <edge source="52" target="53"/>
    <edge source="25" target="26"/>
    <edge source="3" target="4"/>
    <edge source="27" target="28"/>
    <edge source="35" target="36"/>
    <edge source="39" target="40"/>
    <edge source="47" target="48"/>
    <edge source="4" target="5"/>
    <edge source="29" target="30"/>
    <edge source="37" target="38"/>
    <edge source="9" target="10"/>
    <edge source="41" target="42"/>
    <edge source="2" target="3"/>
    <edge source="14" target="15"/>
    <edge source="50" target="51"/>
    <edge source="11" target="12"/>
    <edge source="19" target="20"/>
    <edge source="23" target="24"/>
    <edge source="43" target="44"/>
    <edge source="0" target="1"/>
    <edge source="17" target="18"/>
    <edge source="33" target="34"/>
    <edge source="48" target="49"/>
    <edge source="26" target="27"/>
    <edge source="38" target="39"/>
    <edge source="13" target="14"/>
    <edge source="21" target="22"/>
    <edge source="45" target="46"/>
    <edge source="24" target="25"/>
    <edge source="7" target="8"/>
  </graph>
</graphml>
)GRAPHML";



TEST(BGLGraphTest, LoadsGraphML) {
  BGLGraph graph;
  std::istringstream is(graphml_graph_st0000054);
  graph = BGLGraph::create_from_graphml(is);
  EXPECT_EQ(54, boost::num_vertices(graph));
}

}  // namespace
