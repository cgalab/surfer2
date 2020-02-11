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
#include "SkeletonStructure.h"
#include "BasicTriangulation.h"

void
WavefrontPropagator::
setup_queue(KineticTriangulation& kt) {
  eq = std::make_shared<EventQueue>(kt);
  kt.set_queue(eq);
}

void
WavefrontPropagator::
advance_time() {
  time += increment;

  if (!propagation_complete()) {
    if (no_more_events()) {
      advance_step();
    } else {
      NT want_time = time;
      while (!propagation_complete() && want_time > eq->peak()->get_priority().time()) {
        advance_step();
      }
      time = want_time;
    }
  }
}

void
WavefrontPropagator::
advance_time_next() {
  if (! no_more_events() ) {
    const std::shared_ptr<EventQueueItem> next = eq->peak();
    time = next->get_priority().time();
    if (sk->get_kt().restrict_component()) {
      current_component = peak()->get_priority().t->component;
    }
  }
}

void
WavefrontPropagator::
advance_step() {
  DBG_INDENT_LEVEL_STORE;
  DBG_FUNC_BEGIN(DBG_PROP);

  if (!no_more_events()) {
    const std::shared_ptr<EventQueueItem> next = eq->peak();
    time = next->get_priority().time();
    if (sk->get_kt().restrict_component()) {
      current_component = peak()->get_priority().t->component;
    }
    ++event_ctr_;
    VLOG(2) << " event#" << event_ctr_ << " @ " << CGAL::to_double(time);
    sk->kt.handle_event(next->get_priority());
    DBG(DBG_PROP) << " event#" << event_ctr_ << " handling done.  Processing pending PQ updates.";
    eq->process_pending_updates(time);
    DBG(DBG_PROP) << " event#" << event_ctr_ << " PQ updates done.  Time is now " << CGAL::to_double(time);

    last_event_time = time;
    last_event_component = current_component;
  }

  if (no_more_events()) {
    LOG(INFO) << "All done.";
    finalize();
  } else {
    const Event& next_event = eq->peak()->get_priority();
    DBG(DBG_PROP) << " event#" << (event_ctr_+1) << " will be " << next_event;
    assert((next_event.t->component == current_component && next_event.time() >= last_event_time) ||
            next_event.t->component > current_component);
    if (eq->size() >= 2) {
      DBG(DBG_PROP) << "   event child in heap: " << eq->peak(1)->get_priority();
      if (eq->size() >= 3) {
        DBG(DBG_PROP) << "   event child in heap: " << eq->peak(2)->get_priority();
      }
    }
  }
  DBG_FUNC_END(DBG_PROP);
  DBG_INDENT_LEVEL_CHECK;
}

void
WavefrontPropagator::
advance_to_end() {
  while (!propagation_complete()) {
    advance_step();
  }
}

void
WavefrontPropagator::
do_initial_skips(bool skip_all, unsigned skip_to, const NT& skip_until_time) {
  if (skip_all) {
    advance_to_end();
  } else {
    while (!propagation_complete() &&
           skip_to > event_ctr()+1) {
      advance_step();
    };
  }
  if (skip_until_time > CORE_ZERO) {
    while (!propagation_complete() &&
           (no_more_events() || skip_until_time > peak()->get_priority().time())) {
      advance_step();
    }
    if (skip_until_time > get_time()) {
      advance_time_ignore_event(skip_until_time);
    }
  }
}

void
WavefrontPropagator::
finalize() {
  DBG_FUNC_BEGIN(DBG_PROP);
  assert(no_more_events());
  if (!finalized) {
    DBG(DBG_PROP) << "Calling create_remaining_skeleton_dcel()";
    sk->kt.create_remaining_skeleton_dcel();
    finalized = true;
    current_component = -1;
    DBG(DBG_PROP) << "Finalized.";
  }
  DBG_FUNC_END(DBG_PROP);
}


void
SkeletonStructure::
initialize(int restrict_component) {
  kt.initialize(input, &wavefront_edges, restrict_component);
  assert(kt.triangles_size() > 0);
  wp.setup_queue(kt);
}
