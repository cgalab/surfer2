/**  surfer2 -- a straight skeleton implementation
 *
 *  Copyright 2015 -- 2019 Peter Palfraader
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
#include "SkeletonDCEL.h"
#include "WavefrontEdge.h"


class ParseOffsetSpec
{
  NT time;
  NT step;
  int cnt;

  std::vector<NT> list;

  void
  parse_offset(const std::string&  offset)
  {
    int times = offset.find_first_of('*');
    int b,e;
    if(times != std::string::npos){
      cnt = std::stoi(offset.substr(0,times));
    }else{
      cnt = 1;
      times = -1;
    }
    // we have to trim the string as Core doesn't
    b = offset.find_first_not_of(' ', times+1);
    e = offset.find_last_not_of(' ') + 1;
    step = string_to_maybe_NT(offset.substr(b,e-b));

    for(int i=0; i < cnt; ++i){
      time += step;
      list.emplace_back(time);
    }

  }

  void
  parse_block(const std::string & block, const NT& t = NT(0))
  {
    time = t;
    std::string first, second;
    int plus = block.find_first_of('+');

    if(plus != std::string::npos){
      parse_offset(block.substr(0,plus));
      parse_block(block.substr(plus+1), time);
    }else{
      parse_offset(block);
    }
  }

public:
  const std::vector<NT>&
  operator()(const std::string & input)
  {
    std::string first, second;
    int comma = input.find_first_of(',');

    if(comma != std::string::npos){
      parse_block(input.substr(0,comma));
     (*this)(input.substr(comma+1));
    }else{
      parse_block(input);
    }


    for(NT nt : list){
      std::cout << CGAL::to_double(nt) << " ";
    }
  std::cout << std::endl;

    return list;
  }

};

DEBUG_DECL(
unsigned SkeletonDCELVertexBase::ctr = 0;
unsigned SkeletonDCELHalfedgeBase::ctr = 0;
unsigned SkeletonDCELFaceBase::ctr = 0;
)

SkeletonDCELFace*
SkeletonDCEL::
setup_new_input_edge(const WavefrontEdge * const buddy_wavefront) {
  SkeletonDCELFace * face = new_face();
  SkeletonDCELCbb * ccb = new_outer_ccb();
  SkeletonDCELHalfedge * halfedge = NULL;

  ccb->set_face(face);

  if (buddy_wavefront) { /* buddy wavefront already set up */
    halfedge = (*buddy_wavefront->skeleton_face->outer_ccbs_begin())->opposite();
    assert(halfedge->is_on_outer_ccb());
    assert(halfedge->outer_ccb() == NULL);
  } else {
    halfedge = new_edge();
  }
  halfedge->is_emanating_input_ = true;
  halfedge->set_outer_ccb(ccb);

  face->add_outer_ccb(ccb, halfedge);
  return face;
}

void
SkeletonDCEL::
write_obj(std::ostream& os) const {
  DBG(DBG_SKEL) << " dumping points";
  for (const Point_3& p : points) {
    DBG(DBG_SKEL) << "  dumping point";
    os << "v " << CGAL::to_double(p.x())
       << " " << CGAL::to_double(p.y())
       << " " << CGAL::to_double(p.z()) << std::endl;
  };
  os << std::endl;
  DBG(DBG_SKEL) << " dumping faces";
  for (const SkeletonDCELFace& f : faces) {
    const SkeletonDCELHalfedge* first = * f.outer_ccbs_begin();
    const SkeletonDCELHalfedge* he = first;
    os << "f";
    const Point_3* start_of_points = &*points.begin();
    do {
      const SkeletonDCELVertex* v = he->vertex();
      assert(v);
      if (v->has_null_point()) {
        /* infinity */
      } else {
        const Point_3& p = v->point();
        os << " " << (1 + (&p - start_of_points));
      }

      he = he->next();
    } while (he != first);
    os << std::endl;
  };
}

/** Check if a halfedge intersects an offset distance.
 *
 * returns CGAL::ZERO if no intersection occurs.
 *
 * Otherwise, if the halfedge intersects,
 *   return CGAL::POSITIVE if the halfedge is oriented in the propagation direction,
 *   return CGAL::NEGATIVE if the halfedge is oriented against the propagation direction,
 *   return CGAL::POSITIVE also if the halfedge is horizontal in time.
 * */
CGAL::Sign
SkeletonDCEL::
arc_offset_do_intersect(const NT& offsetting_distance, const Halfedge* const he) const {
  const SkeletonDCELVertex* const tail = he->opposite()->vertex();
  const SkeletonDCELVertex* const head = he->vertex();

  if (tail->has_null_point() || head->has_null_point()) { // ray
    assert(!tail->has_null_point() || !head->has_null_point());

    bool he_is_outgoing = head->has_null_point();
    const NT z = he_is_outgoing ? tail->point().z() : head->point().z();
    if (z <= offsetting_distance) {
      return he_is_outgoing ? CGAL::POSITIVE : CGAL::NEGATIVE;
    }
  } else {
    const NT zt = tail->point().z();
    const NT zh = head->point().z();

    auto [z0, z1] = sorted_tuple(zt, zh);
    if (z0 <= offsetting_distance && offsetting_distance <= z1) {
      if (z0 == z1) return CGAL::POSITIVE;
      return CGAL::sign(zh-zt);
    }
  }
  return CGAL::ZERO;
}

/* Computes the point where the halfedge is at the correct offset.
 *
 * This only works for halfedges not a horizontal in time at the offsetting distance,
 * i.e. no segment intersections.
 */
Point_2
SkeletonDCEL::
arc_offset_get_intersect(const Plane_3& offset_plane, const Halfedge* const he) const {
  DBG_FUNC_BEGIN(DBG_SKEL);
  const Curve& arc = he->curve();
  const Point_3* p = NULL;
  Point_2 p2;
  if (arc.type() == typeid(Segment_3)) {
    const Segment_3& s = boost::get<Segment_3>(arc);
    auto intersect = intersection(s, offset_plane);
    assert(intersect);
    p = boost::get<Point_3>(&*intersect);
    assert(p);
    p2 = project_plane(*p);
  } else {
    DBG(DBG_SKEL) << "arc is a ray";
    assert(arc.type() == typeid(Ray_3));
    const Ray_3& r = boost::get<Ray_3>(arc);
    auto intersect = intersection(r, offset_plane);
    assert(intersect);
    p = boost::get<Point_3>(&*intersect);
    assert(p);
    p2 = project_plane(*p);
  }
  DBG_FUNC_END(DBG_SKEL);
  return p2;
}

std::pair<bool, Segment_2>
SkeletonDCEL::
make_one_offset_segment(const NT& offsetting_distance, const SkeletonDCELHalfedge* start, std::unordered_set<const SkeletonDCELHalfedge*>& visited) const {
  DBG_FUNC_BEGIN(DBG_SKEL);
  std::pair<bool, Segment_2> res;

  const SkeletonDCELHalfedge* segment_start_edge = start;

  DBG(DBG_SKEL) << "Making offset segment starting at " << *start;
  assert(start->vertex()->has_null_point() || start->vertex()->point().z() != offsetting_distance);

  bool inserted = visited.insert(start).second;
  DBG(DBG_SKEL) << "  " << *start << (inserted ? ": marked as visited" : ": already visited before");

  const SkeletonDCELHalfedge* he = start->prev();
  while (1) {
    DBG(DBG_SKEL) << " Looking at " << *he;
    inserted = visited.insert(he).second;
    DBG(DBG_SKEL) << "  " << *he << (inserted ? ": marked as visited" : ": already visited before");
    if (inserted) {
      CGAL::Sign sign = arc_offset_do_intersect(offsetting_distance, he);
      if (sign == CGAL::ZERO) {
        inserted = visited.insert(he->opposite()).second;
        // DBG(DBG_SKEL) << "  " << *he->opposite() << (inserted ? ": marked as visited" : ": already visited before");
        assert(inserted);
      } else {
        if (!he->vertex()->has_null_point() && he->vertex()->point().z() == offsetting_distance) {
          /* the previous vertex had its tail at offsetting_distance. */
          if (he->next() != start) {
            DBG(DBG_SKEL) << "  We skipped over (a presumably parallel in time) segment";
            segment_start_edge = he;
          }
          if (!he->opposite()->vertex()->has_null_point() && he->opposite()->vertex()->point().z() < offsetting_distance) {
            DBG(DBG_SKEL) << "  Node is exactly at offsetting distance and is the local maximum in this face.  No offset segment";
            res.first = false;
            break;
          } else if (!he->opposite()->vertex()->has_null_point() && he->opposite()->vertex()->point().z() == offsetting_distance) {
            DBG(DBG_SKEL) << "  Arc is exactly at offsetting distance.  Making it the offset segment";
            bool inserted2 = visited.insert(he->opposite()).second;
            assert(inserted2);
            res = std::make_pair(true, Segment_2( project_plane(he->vertex()->point()),
                                                  project_plane(he->opposite()->vertex()->point()) ) );
            break;
          } else {
            DBG(DBG_SKEL) << "  Node (or arc) is exactly at offsetting distance but is not the local maximum in this face.  Keep searching";
            assert(sign != CGAL::POSITIVE);
          }
        } else {
          assert(sign == CGAL::POSITIVE);
        }
        if (sign == CGAL::POSITIVE) {
          DBG(DBG_SKEL) << " Offset segment ends at" << *he;

          const Plane_3 offset_plane(CORE_ZERO, CORE_ZERO, CORE_ONE, -offsetting_distance);
          Point_2 source = arc_offset_get_intersect(offset_plane, segment_start_edge);
          Point_2 target = arc_offset_get_intersect(offset_plane, he);
          res = std::make_pair(true, Segment_2(source, target));
          break;
        }
      }
    }
    he = he->prev();
    assert(he != start);
  };

  DBG(DBG_SKEL) << "returning segment " << CGAL_segment(res.second);
  DBG_FUNC_END(DBG_SKEL);
  return res;
}

SkeletonDCEL::OffsetFamily
SkeletonDCEL::
make_offset(const NT& offsetting_distance) const {
  DBG_FUNC_BEGIN(DBG_SKEL);
  DBG(DBG_SKEL) << "making offset at " << CGAL::to_double(offsetting_distance);

  OffsetFamily offsets;

  std::unordered_set<const SkeletonDCELHalfedge*> visited;

  for (const SkeletonDCELHalfedge& he : halfedges) {
    if (visited.find(&he) != visited.end()) {
      // DBG(DBG_SKEL) << "Not considering " << he << " as already visited";
      continue;
    };
    DBG(DBG_SKEL) << "Considering " << he;

    CGAL::Sign sign = arc_offset_do_intersect(offsetting_distance, &he);
    if (sign == CGAL::ZERO) {
      // DBG(DBG_SKEL) << "Not using " << he << " as it does not intersect offset plane";
      bool inserted  = visited.insert(&he).second;
      bool inserted2 = visited.insert(he.opposite()).second;
      assert(inserted);
      assert(inserted2);
    } else if (sign == CGAL::POSITIVE) {
      DBG(DBG_SKEL) << "Not using " << he << " just yet as it is on the wrong side.";
    } else if (!he.vertex()->has_null_point() && he.vertex()->point().z() == offsetting_distance) {
      DBG(DBG_SKEL) << "Not using " << he << " as it has the tip at the offsetting distance."; // XXX we need to catch horizontal arcs somehow still.
    } else {
      DBG(DBG_SKEL) << "Constructing an offset segment at " << he << "; " << *he.prev()->vertex() << "->" << *he.vertex();
      auto [ ok, offset] = make_one_offset_segment(offsetting_distance, &he, visited);
      if (ok) {
        offsets.emplace_back( std::move(offset) );
      } else {
        DBG(DBG_SKEL) << "Not using offset.";
      }
    }
  };

  DBG(DBG_SKEL) << "Num segments: " << offsets.size();
  DBG_FUNC_END(DBG_SKEL);
  return offsets;
}

std::vector<NT>
SkeletonDCEL::
parse_offset_spec(const std::string& offset_spec) {
  DBG_FUNC_BEGIN(DBG_SKEL);
  NT time;
  NT step;
  std::vector<NT> list;

#ifdef _MSC_VER
 ParseOffsetSpec parse;
 list = parse(offset_spec);
#else
 char *spc = strdup(offset_spec.c_str());
  char *saveptr_block = NULL;
  char *saveptr_offset = NULL;
  char *one_block, *one_offset;

  for (one_block = strtok_r(spc, ",", &saveptr_block);
       one_block != NULL;
       one_block = strtok_r(NULL, ",", &saveptr_block)) {
    DBG(DBG_SKEL) << " Got " << one_block;
    time = 0.;
    for (one_offset = strtok_r(one_block, "+", &saveptr_offset);
         one_offset != NULL;
         one_offset = strtok_r(NULL, "+", &saveptr_offset)) {
      int i, cnt = 1;
      char *m = strchr(one_offset,'*');
      DBG(DBG_SKEL) << "  Got " << one_offset;
      if (m) {
        *m = '\0';
        cnt = atoi(one_offset);
        one_offset = m+1;
      }

      while (*one_offset && *one_offset != '.' && (*one_offset < '0' || *one_offset > '9')) {
        ++one_offset;
      }
      char *e = one_offset;
      while (*e && (*e == '.' || (*e >= '0' && *e <= '9'))) {
        ++e;
      }
      if (*e) *e = '\0';

      step = string_to_maybe_NT(one_offset);
      DBG(DBG_SKEL) << "    adding " << cnt << " times " << one_offset << "("<<CGAL::to_double(step)<<")";
      for (i=0; i<cnt; i++) {
        time +=  step;
        DBG(DBG_SKEL) << "      adding at time " << CGAL::to_double(time);
        list.emplace_back(time);
      }
    }
  }

  free(spc);
#endif
  DBG_FUNC_END(DBG_SKEL);
  return list;
}

void
SkeletonDCEL::
write_ipe_segment(std::ostream& os, const Segment_2& s, SegmentType t) {
    os << "  <path cap=\"1\" ";
    switch (t) {
      case SegmentType::INPUT : os << "stroke=\"black\" pen=\"heavier\""; break;
      case SegmentType::ARC   : os << "stroke=\"blue\""; break;
      case SegmentType::OFFSET: os << "stroke=\"black\" pen=\"thin\" dash=\"dotted-narrower\""; break;
    }
    os << ">\n";
    os << "    " << CGAL::to_double(s.source().x()) << " " << CGAL::to_double(s.source().y()) << " m\n";
    os << "    " << CGAL::to_double(s.target().x()) << " " << CGAL::to_double(s.target().y()) << " l\n";
    os << "  </path>\n";
}

void
SkeletonDCEL::
write_ipe(std::ostream& os, const std::string& offset_spec) const {
  const int ray_length = 10;
  std::vector<SkeletonDCEL::OffsetFamily> offsets;
  for (const NT& offset_distance : parse_offset_spec( offset_spec )) {
    offsets.emplace_back(make_offset(offset_distance));
  }

  os << "<?xml version=\"1.0\"?>\n"
        "<!DOCTYPE ipe SYSTEM \"ipe.dtd\">\n"
        "<ipe version=\"70000\" creator=\"surfer2\">\n"
        "<info bbox=\"cropbox\" />\n"

        "<ipestyle name=\"basic\">\n"
        "  <pen name=\"heavier\" value=\"0.8\"/>\n"
        "  <color name=\"blue\" value=\"0 0 1\"/>\n"
        "</ipestyle>\n"
        "<ipestyle name=\"dashstyles\">\n"
        "  <dashstyle name=\"dotted-narrower\" value=\"[0.5 0.5] 0\"/>\n"
        "</ipestyle>\n"
        "<ipestyle name=\"surfer\">\n"
        "  <pen name=\"thin\" value=\"0.2\"/>\n"
        "</ipestyle>\n"

        "<page>\n"
        "<layer name=\"input\" />\n"
        "<layer name=\"sk\" />\n";

  if (offsets.size() > 0) {
    os << "<layer name=\"offsets\" />\n";
    os << "<group layer=\"offsets\">\n";
    for (const auto& family : offsets) {
      os << "<group>\n";
      for (const auto& segment : family) {
        write_ipe_segment(os, segment, SegmentType::OFFSET);
      }
      os << "</group>\n";
    }
    os << "</group>\n";
  };

  os << "<group layer=\"sk\">\n";
  for (auto hit = halfedges_begin(); hit != halfedges_end(); ++hit) {
    if (hit > hit->opposite()) continue; /* Only paint one of every halfedge pair */
    if (hit->is_input()) continue;

    const auto& arc = hit->curve();
    if (arc.type() == typeid(Segment_3)) {
      const Segment_3& s = boost::get<Segment_3>(arc);
      write_ipe_segment(os, project_plane(s), SegmentType::ARC);
    } else {
      assert(arc.type() == typeid(Ray_3));
      const Ray_3& r = boost::get<Ray_3>(arc);
      const Ray_2 r2 = project_plane(r);
      const Segment_2 s2(r2.source(), r2.point(ray_length));
      write_ipe_segment(os, s2, SegmentType::ARC);
    }
  }
  os << "</group>\n";

  os << "<group layer=\"input\">\n";
  for (auto hit = halfedges_begin(); hit != halfedges_end(); ++hit) {
    if (hit > hit->opposite()) continue; /* Only paint one of every halfedge pair */
    if (!hit->is_input()) continue;

    const auto& arc = hit->curve();
    assert(arc.type() == typeid(Segment_3));
    const Segment_3& s = boost::get<Segment_3>(arc);
    write_ipe_segment(os, project_plane(s), SegmentType::INPUT);
  }
  os << "</group>\n";

  os << "</page>\n"
        "</ipe>\n";
}
