/*
 * Copyright (c) 2011, 2012, 2013, 2014, 2015, 2018 Peter Palfrader
 *
 * All Rights reserved.
 */

#pragma once
#include <config.h>

#ifdef HEAP_STATS
  extern unsigned heap_eq_ctr;
  #define STATS_STMT(x) x
#else
  #define STATS_STMT(x) do {} while (0)
#endif

#include "surf.h"

#include <memory>
#include <vector>

#include "tools.h"
#include <CGAL/assertions.h>

template <class PriorityType,
          class HeapItem>
class HeapBase;

#ifndef TESTING_HEAP_IS_ALL_PUBLIC
  #define HEAP_PRIVATE   private
  #define HEAP_PROTECTED protected
#else
  #define HEAP_PRIVATE   public
  #define HEAP_PROTECTED public
#endif

template <class PriorityType>
class HeapItemBase {
    template <class A, class B> friend class HeapBase;

  HEAP_PROTECTED:
    int idx_in_heap;
    PriorityType priority;

  public:
    HeapItemBase() : idx_in_heap(-1) {};
    HeapItemBase(PriorityType p_priority) :
      idx_in_heap(-1),
      priority(p_priority) {};

    const PriorityType& get_priority() const { return priority; }
};

template <class PriorityType,
          class HeapItem = HeapItemBase<PriorityType> >
class HeapBase {
  public:
    typedef typename std::shared_ptr<HeapItem> ElementType;
    typedef typename std::vector< ElementType > ArrayType;

  HEAP_PRIVATE:
    ArrayType v_;

  HEAP_PROTECTED:
    /** given a node's index i, return this node's parent's index.
     *
     * For the root node this operation is not defined. */
    int parent_idx(const int i) const {
      CGAL_precondition(i >= 0 && i < size());
      return (i-1)/2;
    };

    /** given a node's index i, return this node's left child's index.
     *
     * Note that this child might not exist, i.e. the index might be behond
     * the backing array. */
    int left_child_idx(const int i) const {
      CGAL_precondition(i >= 0 && i < size());
      return i*2 + 1;
    };

    /** given a node's index i, return this node's right child's index.
     *
     * Note that this child might not exist, i.e. the index might be behond
     * the backing array. */
    int right_child_idx(const int i) const {
      CGAL_precondition(i >= 0 && i < size());
      return i*2 + 2;
    };

  HEAP_PRIVATE:
    /** swap elements at positions a and b
     */
    void swap_idx(const int a, const int b) {
      CGAL_precondition(a >= 0 && a < size());
      CGAL_precondition(b >= 0 && b < size());
      CGAL_precondition(v_[a]->idx_in_heap == a);
      CGAL_precondition(v_[b]->idx_in_heap == b);

      swap(v_[a], v_[b]);
      v_[a]->idx_in_heap = a;
      v_[b]->idx_in_heap = b;
    };

    void set_from_idx(const int idx, const int src) {
      CGAL_precondition(idx >= 0 && idx < size());
      CGAL_precondition(src >= 0 && src < size());
      //CGAL_precondition(v_[src]->idx_in_heap == src);

      v_[idx] = v_[src];
      v_[idx]->idx_in_heap = idx;
    };

    void set_from_elem(const int idx, ElementType e) {
      CGAL_precondition(idx >= 0 && idx < size());

      v_[idx] = e;
      v_[idx]->idx_in_heap = idx;
    };

    /** restore heap property downwards
     *
     * The two subtrees rooted at the children of root_idx already must
     * satisfy the heap property, only the root may potentially be in
     * violation.
     */
    void sift_down(const int root_idx) {
      DBG_FUNC_BEGIN(DBG_HEAP);
      DBG(DBG_HEAP) << "heapelem#" << root_idx;

      CGAL_precondition(root_idx >= 0 && root_idx < size());

      /*
       * Move the root downwards, swapping it with children, until
       * the entire tree is a heap again.
       */
      ElementType orig_root = v_[root_idx];
      int new_root_idx = root_idx;

      while(1) {
        int left, right, smallest;

        left = left_child_idx(new_root_idx);
        if (left >= size()) {
          break; /* reached the bottom */
        }
        PriorityType& lp = v_[left]->priority;

        right = right_child_idx(new_root_idx);
        if (right >= size()) { /* so, root only has one child. */
          STATS_STMT({if (lp == orig_root->priority) ++heap_eq_ctr;});
          if (lp < orig_root->priority) {
            smallest = left;
          } else {
            break;
          }
        } else {
          PriorityType& rp = v_[right]->priority;
          STATS_STMT({if (lp == orig_root->priority) ++heap_eq_ctr;});
          if (lp < orig_root->priority) {
            STATS_STMT({if (lp == rp) ++heap_eq_ctr;});
            smallest = (lp <= rp) ? left : right;
          } else {
            STATS_STMT({if (rp == orig_root->priority) ++heap_eq_ctr;});
            if (rp < orig_root->priority) {
              smallest = right;
            } else {
              break;
            }
          }
        }
        set_from_idx(new_root_idx, smallest);
        new_root_idx = smallest;
      }
      if (root_idx != new_root_idx) {
        set_from_elem(new_root_idx, orig_root);
      }

      DBG_FUNC_END(DBG_HEAP);
    };

    /** restore heap property upwards
     *
     * The entire tree satisfies the heap property, only the
     * element at child_idx may violate it upwards.  Specifically
     * the subtree rooted at child_idx, including child_idx, is
     * already correct too.
     */
    void sift_up(const int child_idx) {
      DBG_FUNC_BEGIN(DBG_HEAP);
      DBG(DBG_HEAP) << "heapelem#" << child_idx;

      CGAL_precondition(child_idx >= 0 && child_idx < size());

      /*
       * Move the child upwards, swapping it with children, until
       * the entire tree is a heap again.
       */
      ElementType orig_child = v_[child_idx];
      int new_child_idx = child_idx;

      while (new_child_idx != 0) {
        int parent = parent_idx(new_child_idx);
        STATS_STMT({if (v_[parent]->priority == orig_child->priority) ++heap_eq_ctr;});
        if (v_[parent]->priority <= orig_child->priority) {
          break;
        }
        set_from_idx(new_child_idx, parent);
        new_child_idx = parent;
      }
      if (child_idx != new_child_idx) {
        set_from_elem(new_child_idx, orig_child);
      }

      DBG_FUNC_END(DBG_HEAP);
    };


    /** Fix the heap property with respect to a single element whose key has changed.
     */
    void fix_idx(int idx) {
      CGAL_precondition(idx >= 0 && idx < size());

      if (idx != 0 && (v_[idx]->priority <= v_[parent_idx(idx)]->priority)) {
        STATS_STMT({if (v_[idx]->priority == v_[parent_idx(idx)]->priority) ++heap_eq_ctr;});
        sift_up(idx);
      } else {
        sift_down(idx);
      }
    };

    /** Set a new priority for an item
     *
     * Set a new priority for the item idx and fix the heap property with
     * respect to that single element.
     */
    void set_priority(const int idx, const PriorityType& p) {
      CGAL_precondition(idx >= 0 && idx < size());

      v_[idx]->priority = p;
      fix_idx(idx);
    };

    /** Establish the heap property on currently unstructured data.
     */
    void heapify() {
      /* Consider the heap a tree.  Start in the second lowest level and
       * establish the heap property on all these sub-trees (i.e.  put the
       * lowest element in the parent, the two larger ones into the
       * children).
       *
       * Then, go to the next higher level, and for each element establish
       * the heap property of the subtree starting in that element.  Do that
       * by pushing down the element (i.e. switching it with children) until
       * it is smaller than both its children.
       *
       * When we have reached the top level the heap property holds for the
       * entire tree.
       */
      int start_at;

      if (size() <= 1)
        return;
      start_at = (0x1u << log2i(size())) - 2;
      for (int i=start_at; i>=0; --i)
        sift_down(i);

      for (int i=0; i<size(); ++i) {
        v_[i]->idx_in_heap = i;
      }
    };

  HEAP_PROTECTED:
    /** checks whether the heap satisfies the heap property.
     *
     * Runs in linear time!  Useful during debugging as
     * assert(is_heap(h)).
     */
    bool is_heap() const {
      for (int i=size()-1; i>0; --i) {
        int parent = parent_idx(i);
        if (v_[parent]->priority > v_[i]->priority)
          return false;
      }
      for (int i=0; i<size(); ++i) {
        if (v_[i]->idx_in_heap != i) {
          return false;
        }
      }
      return true;
    };
  public:
    HeapBase(const ArrayType& v) :
      v_(v) {
      heapify();
    };

    HeapBase() {};

    void setArray(const ArrayType& v) {
      v_ = v;
      heapify();
    };

    /** remove element at idx from the heap.
     *
     * Returns the element.
     */
    ElementType remove(const int idx) {
      /* Switch the element to be removed with the one at the end of the
       * array.  Then re-establish heap property for the just moved
       * element, sifting up or down as necessary.  (The removed element,
       * now at the end of the array, no longer belongs to the heap.)
       */
      CGAL_precondition(idx >= 0 && idx < size());
      ElementType e = v_[idx];

      swap_idx(idx, size()-1);
      v_.pop_back();

      /* We only need to care about heap property if this wasn't the element
       * at the end anyways */
      if (idx != size()) {
        fix_idx(idx);
      }

      e->idx_in_heap = -1;
      return e;
    };

    /** remove smallest element from the heap.
     *
     * Returns the element.
     */
    ElementType pop() {
      CGAL_precondition(size() > 0);
      return remove(0);
    };

    /** get an element from the heap without removing it.
     */
    const ElementType& peak(const int idx) const {
      CGAL_precondition(idx >= 0 && idx < size());
      return v_[idx];
    }

    /** get the smallest element from the heap without removing it.
     */
    const ElementType& peak() const {
      CGAL_precondition(size() > 0);
      return v_[0];
    }

    void fix_idx(ElementType& e) {
      fix_idx(e->idx_in_heap);
    }

    ElementType drop_element(ElementType& e) {
      return remove(e->idx_in_heap);
    }

    void add_element(ElementType& e) {
      v_.push_back(e);
      v_[size() - 1]->idx_in_heap = size() - 1;
      fix_idx(e->idx_in_heap);
    }

    /** checks whether the heap is empty. */
    bool empty() const {
      return size() == 0;
    };


  HEAP_PROTECTED:
    // const ArrayType& get_v() const { return v_; };
    int size() const { return v_.size(); };
};
