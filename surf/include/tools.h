#pragma once

/* Tools is for stand-alone things that need nothing else of surf or cgal. */

#include <stdint.h>
#include <vector>
#include <tuple>


void my_srand(uint32_t s);
uint32_t my_rand();

/** Compute floor(log2()) of an unsigned integer.
 *
 * Returns -1 for 0.
 *
 * We do this the naive way, requiring log(n) time.
 *
 * If this function ever is part of the critical path consider replacing
 * with something from
 * http://graphics.stanford.edu/~seander/bithacks.html#IntegerLogLookup .
 */
static inline int
log2i(unsigned v) {
  int r = -1;

  while (v > 0) {
    r++;
    v >>= 1;
  };

  return r;
}

template <class T>
void sort_tuple(T& a, T& b) {
  if (a>b) std::swap(a,b);
}

/** returns a triple of indices i0,i1,i2 such that t[i0] <= t[i1] <= t[i2].
 */
template <class T>
std::tuple<unsigned, unsigned, unsigned> indirect_sort_3(const T t[3]) {
  unsigned i0 = 0;
  unsigned i1 = 1;
  unsigned i2 = 2;
  if (t[i0] > t[i1]) std::swap(i0, i1);
  if (t[i1] > t[i2]) std::swap(i1, i2);
  if (t[i0] > t[i1]) std::swap(i0, i1);
  return std::make_tuple(i0, i1, i2);
}


template <typename T>
static T compute_determinant(const T& x0, const T& y0,
                             const T& x1, const T& y1,
                             const T& x2, const T& y2) {
  return T(
    ( x0 * y1
    + x1 * y2
    + x2 * y0
    )
    -
    ( y0 * x1
    + y1 * x2
    + y2 * x0
    )
  );
}

template <class T>
struct FixedVector
  : private std::vector<T> {
  private:
    using Base = std::vector<T>;
    using size_type = typename Base::size_type;
    using value_type = typename Base::value_type;

    using Base::capacity;
  public:
    using const_iterator = typename Base::const_iterator;

    void reserve(size_type new_size) {
      assert(size() == 0);
      Base::reserve(new_size);
    }
    void resize(size_type new_size, const value_type& val) {
      assert(size() == 0);
      Base::resize(new_size, val);
    }
    //using Base::vector;
    //using Base::operator=;
    //using Base::get_allocator;
    using Base::at;
    //using Base::front;
    using Base::back;
    //using Base::clear;
    //using Base::data;
    using Base::begin;
    //using Base::cbegin
    using Base::end;
    //using Base::cend;
    //using Base::empty;
    using Base::size;
    using Base::operator[];

    void emplace_back (value_type&& val) {
      assert(size() < capacity());
      Base::emplace_back(std::forward<value_type>(val));
    }
    void push_back (const value_type& val) {
      assert(size() < capacity());
      Base::push_back(val);
    }
    void push_back (value_type&& val) {
      assert(size() < capacity());
      Base::push_back(std::forward<value_type>(val));
    }
};
