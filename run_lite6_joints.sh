#!/bin/bash
echo "starting joint streaming"
cd /home/user/work/xarm-omniverse/scripts/XArm/client
conda activate xarm
python stream-joints-lite6.py