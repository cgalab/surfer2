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

If you did not clone the source with git, you may have to get easyloggingpp yourself.  You can find it at
https://github.com/cgalab/easyloggingpp

# Build requirements

To build surfer2, you need a c++ toolchain, cmake, and several libraries including CGAL.
On Debian 10 (buster), installing the following packages is sufficient to build surfer2:

  * build-essential
  * cmake
  * libboost-graph-dev
  * libboost-iostreams-dev
  * libcgal-dev
  * libcgal-qt5-dev
  * libqt5opengl5-dev
  * libqt5svg5-dev
  * qtbase5-dev
  * clang


To also build the test suite, you will additionally need `libgtest-dev`.  If
you have an older libgtest-dev (<= 1.7), such as on Ubuntu 18.04, you may need
to explicitly install googletest also.

# Building

To build surfer, run cmake and make:

    mkdir build &&
    cd build &&
    CXX=clang++ cmake -DCMAKE_BUILD_TYPE=Release .. &&
    make

This will create a release build, without expensive assertions.  To build the debug build,
pass `-DCMAKE_BUILD_TYPE=Debug` to cmake, or nothing at all.  If you prefer gcc to clang, don't set
the CXX environment variable when calling cmake.

Other build options are `TEST_SUITE`, which defaults to off (to enable, pass `-DTEST_SUITE=on` to cmake),
and `BUILD_SHARED_LIBS` and `LIB_ONLY` (also off by default).

# Running the command line client or gui

Both the command line client, `surfer`, as well as the gui, `surfgui` take
[GraphML][graphml] files with coordinates as specified in
[Graph-Attributes][graph-attributes] as input.

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
[graph-attributes]: https://github.com/cgalab/format-converter/blob/master/GRAPH-ATTRIBUTES.md

## GUI

The gui has a set of buttons near the top.  They all have mouse-over tool tips.
The block starting with `i` controls layer visibility:

 * `i` toggles visibility of the input PSLG,
 * `I` toggles visibility of the input PSLG labels,
 * `w` toggles visibility of the wavefront,
 * `t` toggles visibility of the triangulation,
 * `T` toggles visibility of the triangulation labels,
 * `s` toggles visibility of the straight skeleton (once finalized),
 * `S` toggles visibility of the straight skeleton labels (once finalized, and only in the debug build).

All of those options can be controlled with the keyboard using Alt+&lt;letter&gt;.  Use Alt+Shift for capitals.

The next two blocks deal with time and even handling.  The functionality behind
these buttons can also be accessed by simply pressing the corresponding keyboard key.

 * `,` moves the time forward to the time of the next event,
 * &lt;backspace&gt; moves the time back to the time of the last event,
 * `b` moves the drawing backwards in time.  Elements are not necessarily consistent in the past.
 * `N` moves the time forwards, including event processing.
 * `M` moves the time forwards, not including event processing.  As such, the future is not necessarily shown correctly.
 * `n` **Moves to the next event time and processes it**.
 * &lt;enter&gt; **Processes all remaining events**.

Lastly, there are a few options that modify the drawing only.  These sometimes help in debugging or investigating how things looked just prior to an event.

 * `-` decrease drawing offset (draw things at a time earlier than now),
 * `+` increase drawing offset (draw things at a time later than now),
 * `=` reset drawing offset.

# License

Surfer is free software.  You may redistribute it and/or modify
it under the terms of the GNU General Public License (v3).
