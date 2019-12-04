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
#include "SkeletonStructure.h"
#include "tools.h"

#include <fstream>
#include <iostream>
#include <getopt.h>
#include <time.h>

#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/stream.hpp>

#include <sys/resource.h>

static const char* short_options = "hc:v::S:D";
static struct option long_options[] = {
  { "help"               , no_argument      , 0, 'h'},
  { "component"          , required_argument, 0, 'c'},
  { "verbose"            , optional_argument, 0, 'v'},
  { "stats-fd"           , required_argument, 0, 'S'},
  { "disable-debug-log"  , no_argument      , 0, 'D'},
  //{ "sk-offset"          , required_argument, 0, 'O'},
  //{ "random-seeed"       , required_argument, 0, 'R'},
  { 0, 0, 0, 0}
};

[[noreturn]]
static void
usage(const char *progname, int err) {
  FILE *f = err ? stderr : stdout;

  fprintf(f,"Usage: %s [options] <INPUT> <OUTPUT>\n", progname);
  fprintf(f,"  Options: --component=0|l|1|r|<n>    Only left (0) or right (1) sided SK (if well defined for input), or component #n in general.\n");
  fprintf(f,"           --stats-fd=<FD>            Enable and print statistics to FD.\n");
#ifdef DEBUG_OUTPUT
  fprintf(f,"           --disable-debug-log        Disable debug level logging (relevant only.\n");
#endif
  /*
  fprintf(f,"  Options: --sk-offset=<offset-spec>  Draw offsets.\n");
  fprintf(f,"           --random-seed=<seed>       Seed for RNG (for debugging).\n");
  fprintf(f,"\n");
  fprintf(f,"  offset-spec = <one-block> [ ',' <one-block> ]\n");
  fprintf(f,"  one-block   = <one-offset> [ '+' one-block ]\n");
  fprintf(f,"  one-offset  = [<cnt> '*' ] <time>\n");
  fprintf(f,"  example: '0.01 + 3*0.025, 0.15' or '10 * 0.025'\n");
  */
  exit(err);
}

static void
do_surf(std::istream& is, std::ostream& os, int restrict_component, int stats_fd = -1) {
  clock_t stage_00 = clock();

  BGLGraph graph = BGLGraph::create_from_graphml(is);
  clock_t stage_01 = clock();
  SkeletonStructure s;
  s.add_graph(graph);
  clock_t stage_02 = clock();
  s.initialize(restrict_component);
  clock_t stage_03 = clock();
  s.wp.advance_to_end();

  clock_t stage_04 = clock();
  clock_t stage_99 = clock();

  s.get_skeleton().write_obj(os);

  if (stats_fd >= 0) {
    boost::iostreams::file_descriptor_sink snk{stats_fd, boost::iostreams::never_close_handle};
    boost::iostreams::stream< boost::iostreams::file_descriptor_sink> stats_os{snk};

    stats_os << std::setprecision(10);
    stats_os << "[SURF] VERSION                "  << VERSIONGIT;
    stats_os << "-" CMAKE_BUILD_TYPE;
    #ifdef NT_USE_DOUBLE
    stats_os << "-NT_USE-DOUBLE";
    #endif
    #ifndef REFINE_TRIANGULATION
    stats_os << "-norefine";
    #endif
    stats_os << std::endl;
    stats_os << "[SURF] INPUT_SIZE             "  << boost::num_vertices(graph) << std::endl;
    stats_os << "[SURF] CPUTIME_PARSEML        "  << ((double) (stage_01-stage_00))/CLOCKS_PER_SEC << std::endl;
    stats_os << "[SURF] CPUTIME_SETUP          "  << ((double) (stage_02-stage_01))/CLOCKS_PER_SEC << std::endl;
    stats_os << "[SURF] CPUTIME_INITKT         "  << ((double) (stage_03-stage_02))/CLOCKS_PER_SEC << std::endl;
    stats_os << "[SURF] CPUTIME_RUN            "  << ((double) (stage_04-stage_03))/CLOCKS_PER_SEC << std::endl;
    stats_os << "[SURF] CPUTIME_TOTAL          "  << ((double) (stage_99-stage_00))/CLOCKS_PER_SEC << std::endl;
    stats_os << "[SURF] CPUTIME_TOTAL_EX_PARSE "  << ((double) (stage_99-stage_01))/CLOCKS_PER_SEC << std::endl;


    struct rusage usage;
    if (getrusage(RUSAGE_SELF, &usage) < 0) {
      LOG(ERROR) << "getrusage() failed: " << strerror(errno);
      exit(1);
    }
    stats_os << "[SURF] MAXRSS                 "  << usage.ru_maxrss << std::endl;

    stats_os << "[SURF] NUM_EVENTS                                 "  << s.get_kt().event_type_counter[int(CollapseType::UNDEFINED)] << std::endl;
    stats_os << "[SURF] NUM_EVENTS_FACE_HAS_INFINITELY_FAST_VERTEX "  << s.get_kt().event_type_counter[int(CollapseType::FACE_HAS_INFINITELY_FAST_VERTEX)] << std::endl;
    stats_os << "[SURF] NUM_EVENTS_TRIANGLE_COLLAPSE               "  << s.get_kt().event_type_counter[int(CollapseType::TRIANGLE_COLLAPSE)] << std::endl;
    stats_os << "[SURF] NUM_EVENTS_CONSTRAINT_COLLAPSE             "  << s.get_kt().event_type_counter[int(CollapseType::CONSTRAINT_COLLAPSE)] << std::endl;
    stats_os << "[SURF] NUM_EVENTS_SPOKE_COLLAPSE                  "  << s.get_kt().event_type_counter[int(CollapseType::SPOKE_COLLAPSE)] << std::endl;
    stats_os << "[SURF] NUM_EVENTS_SPLIT_OR_FLIP_REFINE            "  << s.get_kt().event_type_counter[int(CollapseType::SPLIT_OR_FLIP_REFINE)] << std::endl;
    stats_os << "[SURF] NUM_EVENTS_VERTEX_MOVES_OVER_SPOKE         "  << s.get_kt().event_type_counter[int(CollapseType::VERTEX_MOVES_OVER_SPOKE)] << std::endl;
    stats_os << "[SURF] NUM_EVENTS_CCW_VERTEX_LEAVES_CH            "  << s.get_kt().event_type_counter[int(CollapseType::CCW_VERTEX_LEAVES_CH)] << std::endl;
    #ifdef HEAP_STATS
      stats_os << "[SURF] HEAP_EQUALITIES                            "  << heap_eq_ctr << std::endl;
    #endif
  }
}

int main(int argc, char *argv[]) {
  //std::string skoffset;
  int restrict_component = -1;
  unsigned verbose = 0;
  int stats_fd = -1;
  bool debug_logs = true;

  while (1) {
    int option_index = 0;
    int r = getopt_long(argc, argv, short_options, long_options, &option_index);

    if (r == -1) break;
    switch (r) {
      case 'h':
        usage(argv[0], 0);
        break;

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

      case 'v':
        if (optarg == 0) {
          ++verbose;
        } else {
          char *end_ptr;
          verbose = strtol(optarg, &end_ptr, 10);
          if (*end_ptr != '\0') {
            std::cerr << "Invalid verbosity value" << optarg << std::endl;
            exit(1);
          }
        }
        break;

      case 'S':
        {
          char *end_ptr;
          stats_fd = strtol(optarg, &end_ptr, 10);
          if (*end_ptr != '\0' || stats_fd < 0) {
            std::cerr << "Invalid stats-fd " << optarg << "." << std::endl;
            exit(1);
          }
        }
        break;

      case 'D':
        debug_logs = false;
        break;
      /*
      case 'O':
        skoffset = std::string(optarg);
        break;

      case 'R':
        my_srand(atoi(optarg));
        break;
      */

      default:
        std::cerr << "Invalid option " << (char)r << std::endl;
        exit(1);
    }
  }
  setup_logging(argc, argv, debug_logs);

  el::Loggers::setVerboseLevel(verbose);

  if (argc - optind > 2) {
    usage(argv[0], 1);
  }

  std::istream *in = &std::cin;
  std::ostream *out = &std::cout;
  std::ifstream filestreamin;
  std::ofstream filestreamout;

  if (argc - optind >= 1) {
    std::string fn(argv[optind]);
    if (fn != "-") {
      filestreamin.open(fn);
      in = &filestreamin;
    }
  }
  if (argc - optind >= 2) {
    std::string fn(argv[optind + 1]);
    if (fn != "-") {
      filestreamout.open(fn);
      out = &filestreamout;
    }
  }

  do_surf(*in, *out, restrict_component, stats_fd);
  VLOG(1) << "Did " << CollapseSpec::COUNTER_NT_cmp << " NT comparisons in class CollapseSpec";
  return 0;
}
