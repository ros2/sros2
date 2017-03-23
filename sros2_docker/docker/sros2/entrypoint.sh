#!/bin/bash
set -e

source $HOME/.bashrc

# setup ros
. /opt/nasa/indigo/setup.bash

. /root/sros2_ws/install_isolated/local_setup.bash
exec "$@"
