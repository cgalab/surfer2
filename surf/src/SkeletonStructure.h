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

#include "WavefrontVertex.h"
#include "KineticTriangulation.h"
#include "EventQueue.h"

class SkeletonStructure;

class WavefrontPropagator {
  private:
    SkeletonStructure * const sk;
    std::shared_ptr<EventQueue> eq;
    unsigned event_ctr_ = 0;
    bool finalized = false;

    NT time = CORE_ZERO;
    NT last_event_time = CORE_ZERO;
    NT increment = 0.0005;
    int current_component = -1;
    int last_event_component = -1;

    bool no_more_events() const {
      assert(eq);
      return eq->empty() || peak()->get_priority().type() == CollapseType::NEVER;
    }

  public:
    WavefrontPropagator(SkeletonStructure * const p_sk)
      : sk(p_sk)
    {};
    void setup_queue(KineticTriangulation& kt);

    bool propagation_complete() const {
      return finalized;
    }

    const NT& get_time() const { return time; };
    /** Get the current component we are working on.
     *
     * This is for display purposes only.
     *   -1 if we should show all. if >=0 we are currently working on this component. */
    int get_current_component() const { return current_component; };
    //void set_time(const NT& t) { time = t; };
    void set_increment(const NT& i) { increment = i; };

    /** Move backwards in time */
    void reverse_time() { time -= increment; };

    /** Move forward in time, but ignore any event that may have happened */
    void advance_time_ignore_event() { time += increment; };
    void advance_time_ignore_event(const NT& t) { time = t; };

    /** Move forward in time by the increment, or until the next event and handle it */
    void advance_time();

    /** Move forward in time to the next event but do not handle it yet. */
    void advance_time_next();

    /** Move forward in time to the next event and handle it. */
    void advance_step();

    /** Process events until done. */
    void advance_to_end();

    void reset_time_to_last_event() { time = last_event_time; current_component = last_event_component;  };

    const std::shared_ptr<const EventQueueItem> peak() const {
      assert(eq);
      assert(!eq->empty());
      return eq->peak();
    }

    unsigned event_ctr() const { return event_ctr_; }

    void do_initial_skips(bool skip_all, unsigned skip_to, const NT& skip_until_time);

    /** finish up and create the dcel and whatever else is necessary once the propagation is done. */
    void finalize();
};


class SkeletonStructure {
  friend class WavefrontPropagator;

  private:
    const BasicInput input;
    WavefrontEdgeList wavefront_edges;
    KineticTriangulation kt;

  public:
    WavefrontPropagator wp;

    SkeletonStructure(BasicInput&& input_)
      : input(std::forward<BasicInput>(input_))
      , wp(this)
    { };

    /** copy vertices and edges from a Boost Graph. */
    void initialize(int restrict_component = -1);

    const BasicInput& get_input() const {
      return input;
    }
    const KineticTriangulation& get_kt() const {
      return kt;
    }
    const SkeletonDCEL& get_skeleton() const {
      return kt.get_skeleton();
    }

    //void print_next_info() const;
};
