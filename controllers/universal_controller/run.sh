#!/usr/bin/bash

set -xe

export WEBOTS_HOME=/usr/local/webots
$WEBOTS_HOME/webots-controller --protocol=tcp --ip-address=127.0.0.1  ./universal_controller.py
