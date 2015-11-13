
#include "graspit_bci_plugin/graspit_bci_plugin.h"

extern "C" Plugin* createPlugin() {
  return new graspit_bci_plugin::GraspitBCIPlugin;
}

extern "C" std::string getType() {
  return "graspit_bci_plugin";
}
