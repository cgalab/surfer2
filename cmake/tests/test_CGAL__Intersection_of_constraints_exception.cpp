#include <stdlib.h>

#include <CGAL/Constrained_Delaunay_triangulation_2.h>

#include <CGAL/Exact_predicates_exact_constructions_kernel_with_sqrt.h>
using Kernel  = CGAL::Exact_predicates_exact_constructions_kernel_with_sqrt;
using T = CGAL::Constrained_Delaunay_triangulation_2<Kernel>;

int main(int, char**) {
  try {
  } catch (T::Intersection_of_constraints_exception &err) {
  }
  exit(0);
}
