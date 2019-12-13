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
#include "mainwindow.h"

#include "SkeletonStructure.h"
#include "tools.h"

#include <QApplication>
#include <fstream>
#include <iostream>
#include <getopt.h>

static struct option long_options[] = {
  { "help"        , no_argument      , 0, 'h'},
  { "skip-to"     , required_argument, 0, 's'},
  { "skip-all"    , no_argument      , 0, 'S'},
  { "skip-until",   required_argument, 0, 'T'},
  { "component",    required_argument, 0, 'c'},
  { "sk-offset"   , required_argument, 0, 'O'},
  //{ "random-seeed", required_argument, 0, 'R'},
  { 0, 0, 0, 0}
};

[[noreturn]]
static void
usage(const char *progname, int err) {
  FILE *f = err ? stderr : stdout;

  fprintf(f,"Usage: %s [options] <POLYFILE>\n", progname);
  fprintf(f,"  Options: --skip-to=<n>              Skip to event n at start.\n");
  fprintf(f,"           --skip-all                 Skip until end.\n");
  fprintf(f,"           --skip-until=<time>        Skip until <time>.\n");
  fprintf(f,"           --title=<title>            Set window title.\n"); // by QApplication
  fprintf(f,"           --component=0|l|1|r|<n>    Only left (0) or right (1) sided SK (if well defined for input), or component #n in general.\n");
  fprintf(f,"           --sk-offset=<offset-spec>  Draw offsets.\n");
  //fprintf(f,"           --random-seed=<seed>       Seed for RNG (for debugging).\n");
  fprintf(f,"\n");
  fprintf(f,"  offset-spec = <one-block> [ ',' <one-block> ]\n");
  fprintf(f,"  one-block   = <one-offset> [ '+' one-offset ]\n");
  fprintf(f,"  one-offset  = [<cnt> '*' ] <time>\n");
  fprintf(f,"  example: '0.01 + 3*0.025, 0.15' or '10 * 0.025'\n");
  exit(err);
}

int main(int argc, char *argv[]) {
  setup_logging(argc, argv);
  QApplication a(argc, argv);

  int skip_to = 0;
  bool skip_all = 0;
  std::string skoffset;
  std::string skip_until_time;
  int restrict_component = -1;

  while (1) {
    int option_index = 0;
    //int r = getopt_long(argc, argv, "hs:SO:R:T:", long_options, &option_index);
    int r = getopt_long(argc, argv, "hs:ST:c:", long_options, &option_index);

    if (r == -1) break;
    switch (r) {
      case 'h':
        usage(argv[0], 0);
        break;

      case 'O':
        skoffset = std::string(optarg);
        break;

      case 's':
        skip_to = atoi(optarg);
        break;

      case 'S':
        skip_all = true;
        break;

      case 'T':
        skip_until_time = std::string(optarg);
        break;

      /*
      case 'R':
        my_srand(atoi(optarg));
        break;
        */

      case 'c':
        if (std::string("l") == optarg) {
          restrict_component = 0;
        } else if (std::string("r") == optarg) {
          restrict_component = 1;
        } else {
          char *end_ptr;
          restrict_component = strtol(optarg, &end_ptr, 10);
          if (*end_ptr != '\0') {
            std::cerr << "Invalid component " << optarg << ".  Valid arguments are 0 or 'l' (left) and 1 or 'r' (right) or any number." << std::endl;
            exit(1);
          }
        }
        break;
      default:
        std::cerr << "Invalid option " << (char)r << std::endl;
        exit(1);
    }
  }

  if (argc - optind > 1) {
    usage(argv[0], 1);
  }

  bool use_stdin = true;
  std::ifstream filestream;
  std::string title =
#ifdef CMAKE_BUILD_TYPE
    "surfGUI (" CMAKE_BUILD_TYPE ")";
#else
    "surfGUI";
#endif
  if (argc - optind == 1) {
    std::string fn(argv[optind]);
    if (fn != "-") {
      title += " [" + fn + "]";
      filestream.open(fn);
      if (! filestream.is_open()) {
        LOG(ERROR) << "Failed to open " << fn << ": " << strerror(errno);
        exit(1);
      }
      use_stdin = false;
    }
  }
  std::istream &in = use_stdin ? std::cin : filestream;

  MainWindow w(title, in, skip_to, skip_all, skip_until_time, skoffset, restrict_component);
  w.show();

  return a.exec();
  return 0;
}
