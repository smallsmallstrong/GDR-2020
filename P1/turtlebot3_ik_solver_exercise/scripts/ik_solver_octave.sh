#!/bin/bash

source $ROSWSS_BASE_SCRIPTS/helper/helper.sh

echo_note "Waiting for rosbridge_websocket to start up..."
until rosnode list | grep rosbridge_websocket &>/dev/null ; do sleep 1; done
echo_info "rosbridge_websocket has been detected. Starting up Octave."

cd $1/octave
octave solve_ik.m
