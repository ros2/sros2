#!/bin/bash
set -e

source $HOME/.bashrc

# setup ros
. /root/sros2_ws/install_isolated/local_setup.bash
exec "$@"
