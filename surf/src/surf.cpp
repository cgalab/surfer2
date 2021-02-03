/**  surfer2 -- a straight skeleton implementation
 *
 *  Copyright 2018, 2019 Peter Palfrader
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
#include "surf.h"

INITIALIZE_EASYLOGGINGPP

unsigned DBG_INDENT_CTR = 0;

void
setup_logging(int argc, char* argv[], bool debugLogs) {
  START_EASYLOGGINGPP(argc, argv);

  el::Configurations defaultConf;
  defaultConf.setGlobally(el::ConfigurationType::Format, "%datetime{%H%m%s.%g} %levshort %msg");
  if (! debugLogs) {
    defaultConf.set(el::Level::Debug, el::ConfigurationType::Enabled, "false");
  }
  el::Loggers::reconfigureAllLoggers(defaultConf);
  el::Loggers::addFlag( el::LoggingFlag::ColoredTerminalOutput );
}

#ifdef HEAP_STATS
unsigned heap_eq_ctr = 0;
#endif
