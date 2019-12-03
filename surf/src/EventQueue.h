#pragma once

#include "surf.h"

#include "CollapseSpec.h"
#include "Heap.h"

class Event : public CollapseSpec {
  public:
    const KineticTriangle * const t;
    Event(const KineticTriangle* t, const NT& now);

    void update_collapse(const NT& now);

    friend std::ostream& operator<<(std::ostream& os, const Event& e);
};

class EventQueueItem : public HeapItemBase <Event> {
  private:
  public:
    EventQueueItem(const KineticTriangle* t, const NT& now)
      : HeapItemBase<Event>(Event(t, now))
    {};

    void update_priority(const NT& now) {
      priority.update_collapse(now);
    }
};


class EventQueue : private HeapBase <Event, EventQueueItem> {
  private:
    using Base = HeapBase <Event, EventQueueItem>;

  private:
    std::vector<const KineticTriangle *> need_update;
    std::vector<const KineticTriangle *> need_dropping;
    FixedVector<bool> tidx_in_need_dropping;
    FixedVector<bool> tidx_in_need_update;

    FixedVector<ElementType> tidx_to_qitem_map;

    void tidx_to_qitem_map_add(const KineticTriangle * t, ElementType qi);
    void drop_by_tidx(unsigned tidx);
    void update_by_tidx(unsigned tidx, const NT& now);
    void assert_no_pending() const;
  public:
    EventQueue(const KineticTriangulation& kt);

    /* we /could/ make this const, and our heap a mutable
     * object attribute and the vectors also mutable.
     * At which point pretty much everything in here is
     * declared mutable, so let's just not.
     */
    const ElementType& peak() const {
      assert_no_pending();
      return Base::peak();
    }
    const ElementType& peak(int idx) const {
      assert_no_pending();
      return Base::peak(idx);
    }
    using Base::size;
    using Base::empty;

    void process_pending_updates(const NT& now);

    void needs_update(const KineticTriangle * t, bool may_have_valid_collapse_spec = false);
    void needs_dropping(KineticTriangle * t);

    bool in_needs_update(const KineticTriangle * t) const;
    bool in_needs_dropping(const KineticTriangle * t) const;

    bool is_valid_heap() const;
};
