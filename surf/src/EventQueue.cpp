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
#include "EventQueue.h"
#include "KineticTriangulation.h"
#include "KineticTriangle.h"

Event::
Event(const KineticTriangle* p_t, const NT& now)
  : CollapseSpec(p_t->get_collapse(now))
  , t(p_t)
{
      // DBG(DBG_KT) << " collapse time for f#" << f->getId() << " is " << get_printable_time() << " (" << type() << ")";
  DBG_FUNC_BEGIN(DBG_EVENTQ);
  DBG(DBG_EVENTQ) << "making " << *this << ".";
  DBG_FUNC_END(DBG_EVENTQ);
}

void
Event::
update_collapse(const NT& now) {
  DBG_FUNC_BEGIN(DBG_EVENTQ);

  CollapseSpec::operator=( t->get_collapse(now) );

  DBG_FUNC_END(DBG_EVENTQ);
}

EventQueue::
EventQueue(const KineticTriangulation& kt) {
  ArrayType a;
  tidx_to_qitem_map.resize(kt.triangles_size(), NULL);

  tidx_in_need_dropping.resize(kt.triangles_size(), false);
  tidx_in_need_update.resize(kt.triangles_size(), false);

  for (auto t = kt.triangles_begin(); t != kt.triangles_end(); ++t) {
    auto qi = std::make_shared<EventQueueItem>(&*t, CORE_ZERO);
    a.emplace_back(qi);
    tidx_to_qitem_map_add(&*t, qi);
  }
  setArray(a);
  DBG(DBG_EVENTQ) << "heap array:";
  for (int i=0; i < size(); ++i) {
    DBG(DBG_EVENTQ) << " item " << i << ": " << peak(i)->get_priority();
  }
  const auto x = peak();
  DBG(DBG_EVENTQ) << "top: " << x->get_priority();
#if defined (DEBUG_EXPENSIVE_PREDICATES) && DEBUG_EXPENSIVE_PREDICATES >= 1
  if (! is_valid_heap() ) {
    LOG(ERROR) << "Heap is not valid!";
    exit(EXIT_INIT_INVALID_HEAP);
  }
#endif
}

void
EventQueue::
tidx_to_qitem_map_add(const KineticTriangle * t, ElementType qi) {
  unsigned id = t->id;
  /*
  if (id >= tidx_to_qitem_map.size()) {
    tidx_to_qitem_map.resize(id+1);
  };
  */
  assert(id < tidx_to_qitem_map.size());
  tidx_to_qitem_map[id] = qi;
}

void
EventQueue::
drop_by_tidx(unsigned tidx) {
  // DBG_FUNC_BEGIN(DBG_EVENTQ);
  DBG(DBG_EVENTQ) << "tidx: " << tidx;

  assert(tidx_in_need_dropping[tidx]);
  auto qi = tidx_to_qitem_map.at(tidx);
  assert(NULL != qi);
  drop_element(qi);
  tidx_to_qitem_map[tidx] = NULL;
  tidx_in_need_dropping[tidx] = false;

  // DBG_FUNC_END(DBG_EVENTQ);
}

void
EventQueue::
update_by_tidx(unsigned tidx, const NT& now) {
  // DBG_FUNC_BEGIN(DBG_EVENTQ);
  DBG(DBG_EVENTQ) << "tidx: " << tidx << "; now: " << CGAL::to_double(now);

  assert(tidx_in_need_update[tidx]);
  auto qi = tidx_to_qitem_map.at(tidx);
  assert(NULL != qi);
  qi->update_priority(now);
  fix_idx(qi);
  tidx_in_need_update[tidx] = false;

  // DBG_FUNC_END(DBG_EVENTQ);
}


void
EventQueue::
process_pending_updates(const NT& now) {

  for (auto t : need_dropping) {
    assert(t);
    drop_by_tidx(t->id);
  }
  need_dropping.clear();

  for (auto t : need_update) {
    assert(t);
    update_by_tidx(t->id, now);
  }
  need_update.clear();

  assert_no_pending();
}

void
EventQueue::
assert_no_pending() const {
  assert(need_update.empty());
  assert(need_dropping.empty());
}

/** Mark a triangle as needing an update in the priority queue.
 *
 * In general, this implies its collapse spec has become invalidated,
 * however during event refinement we may actually already have set
 * the new collapse spec and it's valid, in which case we need
 * to pass bool may_have_valid_collapse_spec to appease the assertion.
 */
void
EventQueue::
needs_update(const KineticTriangle * t, bool may_have_valid_collapse_spec) {
  // DBG_FUNC_BEGIN(DBG_EVENTQ);
  DBG(DBG_EVENTQ) << "t" << t;

  assert(tidx_in_need_update.size() > t->id);
  assert(!t->is_collapse_spec_valid() || may_have_valid_collapse_spec);
  // during refinement, the same triangle may be tagged as needs_update multiple times.
  if (! tidx_in_need_update[t->id]) {
    tidx_in_need_update[t->id] = true;
    need_update.push_back(t);
  }

  assert(!tidx_in_need_dropping[t->id]); /* Can't drop and update both */
  // DBG_FUNC_END(DBG_EVENTQ);
}

void
EventQueue::
needs_dropping(KineticTriangle * t) {
  // DBG_FUNC_BEGIN(DBG_EVENTQ);
  DBG(DBG_EVENTQ) << "t" << t;

  assert(tidx_in_need_dropping.size() > t->id);
  assert(t->is_dying());
  t->set_dead();

  assert(!tidx_in_need_dropping[t->id]);
  tidx_in_need_dropping[t->id] = true;

  need_dropping.push_back(t);

  assert(!tidx_in_need_update[t->id]); /* Can't drop and update both */
  // DBG_FUNC_END(DBG_EVENTQ);
}

bool
EventQueue::
in_needs_update(const KineticTriangle * t) const {
  return tidx_in_need_update[t->id];
}

bool
EventQueue::
in_needs_dropping(const KineticTriangle * t) const {
  return tidx_in_need_dropping[t->id];
}

/** checks whether the heap satisfies the heap property.
 *
 * Runs in linear time, so should only be used as debugging tool.
 */
bool
EventQueue::
is_valid_heap() const {
#ifndef NT_USE_DOUBLE
  for (int i=size()-1; i>0; --i) {
    int parent = parent_idx(i);
// (v1-v2).Rep()->getExactSign()
    NT delta = peak(parent)->get_priority().time() - peak(i)->get_priority().time();
    if (delta.Rep()->getSign() != delta.Rep()->getExactSign()) {
      LOG(ERROR) << "Sign mismatch at heap item " << parent << " vs. " << i;
      DBG(DBG_EVENTQ) << " item " << parent << ": " << peak(i)->get_priority();
      DBG(DBG_EVENTQ) << " item " << i << ": " << peak(parent)->get_priority();
      DBG(DBG_EVENTQ) << " delta is " << delta;
      DBG(DBG_EVENTQ) << " sign is " << delta.Rep()->getSign();
      DBG(DBG_EVENTQ) << " exact sign is " << delta.Rep()->getExactSign();
      return false;
    }
    if (peak(parent)->get_priority() > peak(i)->get_priority()) {
      LOG(ERROR) << "Mismatch at heap item " << parent << " vs. " << i;
      DBG(DBG_EVENTQ) << " item " << parent << ": " << peak(i)->get_priority();
      DBG(DBG_EVENTQ) << " item " << i << ": " << peak(parent)->get_priority();
      return false;
    }
  }
#endif
  return Base::is_heap();
}

std::ostream&
operator<<(std::ostream& os, const Event& e) {
  os << "Event in " << e.t->get_name() << " " << CollapseSpec(e);
  return os;
}

