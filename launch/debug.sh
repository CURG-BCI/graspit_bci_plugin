#!/bin/bash


export GRASPIT=~/ros/bci_project/graspit_bci_ws/src/graspit-ros/graspit/graspit_source
export CGDB_MODEL_ROOT=${GRASPIT}/cgdb
export GRASPIT_PLUGIN_DIR=~/ros/bci_project/graspit_bci_ws/devel/lib
export SPRITES_DIR=~/ros/bci_project/graspit_bci_ws/src/graspit_bci_plugin/sprites/
export LOG_DIR=~/ros/bci_project/graspit_bci_ws/src/graspit_bci_plugin/logs/

gdb ~/ros/graspit_bci_ws/devel/lib/graspit/graspit -p libgraspit_bci_plugin

#r -p libgraspit_bci_plugin
