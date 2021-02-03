/**  surfer2 -- a straight skeleton implementation
 *
 *  Copyright 2015 -- 2019 Peter Palfrader
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
#define TESTING_HEAP_IS_ALL_PUBLIC 1

#include "Heap.h"
#include "gtest/gtest.h"

#include <cstddef>

static void test_heap_basic(void *);
static void test_heap_heapify(void *);
static void test_heap_remove(void *);
static void test_heap_pop(void *);

class HeapItem :
  public HeapItemBase<double> {
  friend void test_heap_pop(void *bla);

  template <class A, class B> friend class HeapBase;
  public:
    int data;
    HeapItem(int p_data, double p) :
      HeapItemBase<double>(p),
      data(p_data) {};
};

class Heap : public HeapBase<double, HeapItem> {
  friend void test_heap_basic(void *bla);
  friend void test_heap_heapify(void *bla);
  friend void test_heap_remove(void *bla);
  friend void test_heap_pop(void *bla);

  public:
    typedef HeapBase<double, HeapItem> Parent;
    typedef typename Parent::ArrayType ArrayType;

  public:
    Heap(const ArrayType& v) : Parent(v) {};
};

typedef typename Heap::ArrayType HeapArrayType;

TEST(HeapTest, Basic) {
  HeapArrayType a;
  a.push_back(std::make_shared<HeapItem>(1, 9.4));
  a.push_back(std::make_shared<HeapItem>(2, 3.3));
  a.push_back(std::make_shared<HeapItem>(3, 5.1));


  Heap h(a);

  EXPECT_EQ(a.size(), 3);
  ASSERT_TRUE(h.is_heap());
  EXPECT_EQ(h.size(), 3);
  EXPECT_EQ(h.v_[0]->data, 2);
  EXPECT_EQ(h.v_[1]->data, 1);
  EXPECT_EQ(h.v_[2]->data, 3);

  h.swap_idx(0,1);
  ASSERT_FALSE(h.is_heap());
}

static Heap
heap_test_generate(int size) {
  HeapArrayType a;
  for (int i=0; i<size; ++i) {
    a.push_back(std::make_shared<HeapItem>(i, (1.0*size) / (rand() % (100 * size)) ));
  }
  return Heap(a);
}

TEST(HeapTest, Heapify) {
  int size=200000;
  Heap h = heap_test_generate(size);
  EXPECT_EQ(h.size(), size);
  ASSERT_TRUE(h.is_heap());
}

TEST(HeapTest, Remove) {
  int size=2000;
  Heap h = heap_test_generate(size);
  EXPECT_EQ(h.size(), size);
  ASSERT_TRUE(h.is_heap());
  for (;size > 0; --size) {
    EXPECT_EQ(size, h.size());

    int remove = rand() % size;
    h.remove(remove);

    ASSERT_TRUE(h.is_heap());
  }
}

TEST(HeapTest, Pop) {
  int size=2000;
  Heap h = heap_test_generate(size);
  double v = 0;
  EXPECT_EQ(h.size(), size);
  ASSERT_TRUE(h.is_heap());
  for (;size > 0; --size) {
    EXPECT_EQ(size, h.size());
    auto e = h.pop();

    ASSERT_LE(v, e->priority);
    ASSERT_TRUE(h.is_heap());

    v = e->priority;
  }
}
