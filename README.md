# Surfer2

Surfer2 is an implementation of Aichholzer and Aurenhammer's triangulation
based straight skeleton algorithm using CGAL.  It has been developed at
the University of Salzburg's Computational Geometry and Applications Lab.

# Obtaining the source code

Clone the git repository:
The logging class is included via a git submodule, so either clone the source
using `

    git clone --recurse-submodules https://github.com/cgalab/surfer2

or, if you already have a checkout without the submodule, go to the working copy and

    git submodule update --init --recursive

If all works well, you should see files both at the top level of your working copy as well as in `surf/easyloggingpp`.

# Build requirements

To build surfer2, you need a c++ toolchain, cmake, and several libraries including CGAL.
On Debian 10 (buster), installing the following packages is sufficient to build surfer2:

  * build-essential
  * cmake
  * libboost-graph-dev
  * libboost-iostreams-dev
  * libcgal-dev
  * libcgal-qt5-dev
  * libgtest-dev
  * libqt5opengl5-dev
  * libqt5svg5-dev
  * qtbase5-dev
  * clang

# Building

To build surfer, run cmake and make:

    mkdir build &&
    cd build &&
    CXX=clang++ cmake -DCMAKE_BUILD_TYPE=Release .. &&
    make

This will create a release build, without expensive assertions.  To build the debug build,
pass `-DCMAKE_BUILD_TYPE=Debug` to cmake, or nothing at all.

# Running the command line client or gui

Both the command line client, `surfer`, as well as the gui, `surfgui` take
[GraphML][graphml] files as input.

    ./cc/surfer ../test-data/srpg0000025.graphml

    ./gui/surfgui ../test-data/srpg0000028.graphml


They accept several options, one of them is `--help` which prints a list of all
options.  The `--component=<component>` option restricts computation to
component number `<component>`, an integer number starting at 0.  For polygonal
input, `l` and `r` are accepted for (left and right).  The default is `-1`, which
computes all components' straight skeleton, one after the other.  A value of `-2`
propagates the wavefront in all components at the same time.

To create a .graphml file from a .line or .ipe file use

    ord-format <linefile>

from the [format-converter repository][format-converter].

[graphml]: http://graphml.graphdrawing.org/
[format-converter]: https://github.com/cgalab/format-converter

# License

Surfer is free software.  You may redistribute it and/or modify
it under the terms of the GNU General Public License (v3).
