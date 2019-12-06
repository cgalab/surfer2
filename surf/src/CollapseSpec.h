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

#include "cgaltools.h"

enum class EdgeCollapseType : short {
  UNDEFINED = 1,
  PAST,   /* endpoints moving away from another */
  FUTURE, /* endpoints moving towards one another */
  ALWAYS, /* endpoints moving moving in parrallel and conincident */
  NEVER,  /* endpoints moving moving in parrallel but not conincident */
  };
std::ostream& operator<<(std::ostream& os, const EdgeCollapseType a);

class EdgeCollapseSpec {
  private:
    EdgeCollapseType type_;
    NT time_;

  public:
    EdgeCollapseSpec()
      : type_(EdgeCollapseType::UNDEFINED)
    {}
    EdgeCollapseSpec(EdgeCollapseType type)
      : type_(type)
    {
      assert(type_ == EdgeCollapseType::PAST ||
             type_ == EdgeCollapseType::ALWAYS ||
             type_ == EdgeCollapseType::NEVER );
    };
    EdgeCollapseSpec(const EdgeCollapseType& type,
                 const NT& time)
      : type_(type)
      , time_(time)
    {
      assert(type_ == EdgeCollapseType::FUTURE);
    };

    EdgeCollapseType type() const { return type_; }
    const NT& time() const { return time_; };
    double get_printable_time() const { return CGAL::to_double(time_); }
};
std::ostream& operator<<(std::ostream& os, const EdgeCollapseSpec& s);



enum class CollapseType : short {
  UNDEFINED = 1,
  FACE_HAS_INFINITELY_FAST_VERTEX_OPPOSING, /** This triangle has a vertex
                       which is between parallel, opposing wavefront elements
                       that have crashed into each other and their intersection
                       is now a line segment. */
  FACE_HAS_INFINITELY_FAST_VERTEX_WEIGHTED, /** This triangle has a vertex which is
                       between parallel adjacent wavefront elements that have
                       different weights but move in the same direction. */
  TRIANGLE_COLLAPSE,
  // NEIGHBORING_TRIANGLE_COLLAPSES,  /* do we ever need to handle this */
  CONSTRAINT_COLLAPSE,
  SPOKE_COLLAPSE,          /* two non-incident vertices become incident,
                              splitting the wavefront here.
                              UNUSED except in get_generic */
  SPLIT_OR_FLIP_REFINE,    /* vertex moves onto supporting line of constraint,
                              can refine event type when it comes to it.  */
  VERTEX_MOVES_OVER_SPOKE, /* vertex moves into spoke (triangulation edge interior),
                              flip event */
  CCW_VERTEX_LEAVES_CH,    /* the ccw vertex of the infinite vertex in an
                              unbounded triangle leaves the convex hull of the
                              wavefront polygon */
// GENERIC_FLIP_EVENT,
  INVALID_EVENT, /* The triangle will collapse at this time, but we should
                    never see this as prior events should have rebuilt the
                    triangulation in some way.  If this is the next event,
                    something went wrong. */
  NEVER, /* Leave this one last.  It serves also as a counter! */
  };
std::ostream& operator<<(std::ostream& os, const CollapseType a);

class CollapseSpec {
  public:
    static unsigned COUNTER_NT_cmp;

  private:
    CollapseType type_;
    NT time_;

    /* extra info */
    // for all collapses listed in requires_relevant_edge(), such as CONSTRAINT_COLLAPSE
    int relevant_edge_ = -1;
    // for VERTEX_MOVES_OVER_SPOKE
    NT secondary_key_; // Higher number is more important
                       // - for flip events: longest spoke: squared length
                       // - for weighted infinite one: higher edge speed (if faster edge wins)

    static bool requires_relevant_edge(const CollapseType& type) {
      return
         type == CollapseType::FACE_HAS_INFINITELY_FAST_VERTEX_WEIGHTED ||
         type == CollapseType::CONSTRAINT_COLLAPSE ||
         type == CollapseType::SPOKE_COLLAPSE ||
         type == CollapseType::SPLIT_OR_FLIP_REFINE ||
         type == CollapseType::VERTEX_MOVES_OVER_SPOKE ||
         type == CollapseType::CCW_VERTEX_LEAVES_CH ||
         false;
    }
    static bool requires_relevant_edge_plus_secondary_key(const CollapseType& type) {
      return
         type == CollapseType::FACE_HAS_INFINITELY_FAST_VERTEX_WEIGHTED ||
         type == CollapseType::VERTEX_MOVES_OVER_SPOKE ||
         false;
    }
  public:
    const int component;

    bool requires_relevant_edge() const {
      return requires_relevant_edge(type_);
    }
    bool requires_relevant_edge_plus_secondary_key() const {
      return requires_relevant_edge_plus_secondary_key(type_);
    }


  public:
    //CollapseSpec(const CollapseSpec&) = delete;
    //CollapseSpec(CollapseSpec&&) = default;
    //CollapseSpec& operator = (CollapseSpec&&) = default;
    CollapseSpec& operator =(const CollapseSpec& o) {
      assert(component == o.component);
      type_ = o.type_;
      time_ = o.time_;
      relevant_edge_ = o.relevant_edge_;
      secondary_key_ = o.secondary_key_;
      return *this;
    }

    CollapseSpec(int p_component)
      : type_(CollapseType::UNDEFINED)
      , component(p_component)
    {}
    CollapseSpec(int p_component,
                 CollapseType type)
      : type_(type)
      , component(p_component)
    {
      assert(type_ == CollapseType::NEVER);
    };
    CollapseSpec(int p_component,
                 const CollapseType& type,
                 const NT& time)
      : type_(type)
      , time_(time)
      , component(p_component)
    {
      assert(type_ != CollapseType::UNDEFINED);
      assert(type_ != CollapseType::NEVER);
      assert(! requires_relevant_edge(type_));
    };
    CollapseSpec(int p_component,
                 const CollapseType& type,
                 const NT& time,
                 int relevant_edge)
      : type_(type)
      , time_(time)
      , relevant_edge_(relevant_edge)
      , component(p_component)
    {
      assert(requires_relevant_edge(type_));
      assert(!requires_relevant_edge_plus_secondary_key(type_));
      assert(0 <= relevant_edge_ && relevant_edge_ < 3);
    };
    CollapseSpec(int p_component,
                 const CollapseType& type,
                 const NT& time,
                 int relevant_edge,
                 const NT& secondary_key)
      : type_(type)
      , time_(time)
      , relevant_edge_(relevant_edge)
      , secondary_key_(secondary_key)
      , component(p_component)
    {
      assert(requires_relevant_edge(type_));
      assert(requires_relevant_edge_plus_secondary_key(type_));
      assert(0 <= relevant_edge_ && relevant_edge_ < 3);
    };
    CollapseSpec(int p_component,
                 const EdgeCollapseSpec& edge_collapse,
                 int relevant_edge)
      : type_(edge_collapse.type() == EdgeCollapseType::FUTURE ? CollapseType::CONSTRAINT_COLLAPSE :
              edge_collapse.type() == EdgeCollapseType::ALWAYS ? CollapseType::CONSTRAINT_COLLAPSE :
                                                                 CollapseType::NEVER)
      , time_(type_ == CollapseType::CONSTRAINT_COLLAPSE ? edge_collapse.time() : CORE_ZERO)
      , relevant_edge_(type_ == CollapseType::CONSTRAINT_COLLAPSE ? relevant_edge : 0)
      , component(p_component)
    {
      assert(edge_collapse.type() == EdgeCollapseType::FUTURE ||
             edge_collapse.type() == EdgeCollapseType::ALWAYS ||
             edge_collapse.type() == EdgeCollapseType::NEVER  ||
             edge_collapse.type() == EdgeCollapseType::PAST);
      assert(0 <= relevant_edge_ && relevant_edge_ < 3);
    };
    /*
    CollapseSpec(const CollapseSpec &o)
      : type_(o.type_)
      , time_(o.time_)
    {}
    */

    CollapseType type() const { return type_; }
    const NT& time() const { return time_; };
    double get_printable_time() const { return CGAL::to_double(time_); }
    double get_printable_secondary_key() const { return CGAL::to_double(secondary_key_); }
    int relevant_edge() const {
      assert(requires_relevant_edge());
      assert(0 <= relevant_edge_ && relevant_edge_ < 3);
      return relevant_edge_;
    }
    bool allows_refinement_to(const CollapseSpec &o) const {
      assert(time_ == o.time_);
      if (type_ == CollapseType::SPLIT_OR_FLIP_REFINE) {
        if (o.type_ == CollapseType::VERTEX_MOVES_OVER_SPOKE ||
            o.type_ == CollapseType::SPOKE_COLLAPSE) {
          if (relevant_edge_ != o.relevant_edge_) {
            return true;
          }
        }
      }
      return false;
    }

  private:
    static CGAL::Comparison_result compare_NT(const NT& a, const NT& b) {
      ++COUNTER_NT_cmp;
      if (a < b) {
        return CGAL::SMALLER;
      } else if (a > b) {
        return CGAL::LARGER;
      } else {
        return CGAL::EQUAL;
      }
    }

    CGAL::Comparison_result compare(const CollapseSpec &o) const {
      assert(  type_ != CollapseType::UNDEFINED);
      assert(o.type_ != CollapseType::UNDEFINED);

      if (type_ == CollapseType::NEVER) {
        if (o.type_ == CollapseType::NEVER) {
          return CGAL::EQUAL;
        } else {
          return CGAL::LARGER;
        }
      } else if (o.type_ == CollapseType::NEVER) {
        return CGAL::SMALLER;
      }

      if (component < o.component) {
        return CGAL::SMALLER;
      } else if (component > o.component) {
        return CGAL::LARGER;
      }

      auto c = compare_NT(time_, o.time_);
      if (c == CGAL::EQUAL) {
        if (type_ < o.type_) {
          c = CGAL::SMALLER;
        } else if (type_ > o.type_) {
          c = CGAL::LARGER;
        } else if (requires_relevant_edge_plus_secondary_key()) {
          c = -compare_NT(secondary_key_, o.secondary_key_);
        }
      }
      return c;
    }

  public:
    bool operator< (const CollapseSpec &o) const { return compare(o) == CGAL::SMALLER; }
    bool operator> (const CollapseSpec &o) const { return compare(o) == CGAL::LARGER; }
    bool operator>= (const CollapseSpec &o) const { return !(*this < o); };
    bool operator<= (const CollapseSpec &o) const { return !(*this > o); };
    bool operator== (const CollapseSpec &o) const { return compare(o) == CGAL::EQUAL; }
    bool operator!= (const CollapseSpec &o) const { return !(*this == o); };
};
std::ostream& operator<<(std::ostream& os, const CollapseSpec& s);
