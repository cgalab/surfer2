# Surfer2

Surfer2 is an implementation of Aichholzer and Aurenhammer's triangulation
based straight skeleton algorithm using CGAL.  It has been developed at
the University of Salzburg's Computational Geometry and Applications Lab.

# Obtaining the source code

Clone the git repository:
The logging class is included via a git submodule, so either clone the source
using `

    git clone --recurse-submodules git@gitlab.cosy.sbg.ac.at:cg/ord/surfer2.git

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

# building

To build surfer, run cmake and make:

    mkdir build &&
    cd build &&
    CXX=clang++ cmake -DCMAKE_BUILD_TYPE=Release .. &&
    make

This will create a release build, without expensive assertions.  To build the debug build,
pass `-DCMAKE\_BUILD\_TYPE=Debug` to cmake, or nothing at all.

# running the gui

    ./gui/surfgui foo.graphml

To create a .graphml file from a .line or .ipe file use

    ord-format <linefile>

from the format-converter repository.
