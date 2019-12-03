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
