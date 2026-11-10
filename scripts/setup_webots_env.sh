#!/usr/bin/env bash
# Webots Environment Setup
export WEBOTS_HOME="/data/yhwu/robot/webots"
export PATH="/data/yhwu/miniconda3/envs/memrec/bin:$WEBOTS_HOME:/data/yhwu/robot/tools/xvfb/usr/bin:$PATH"
export PYTHONPATH="$WEBOTS_HOME/lib/controller/python${PYTHONPATH:+:$PYTHONPATH}"
