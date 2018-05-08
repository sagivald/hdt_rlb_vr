#!/bin/bash

terminator -e "Visualizer & roslaunch hdt_arm_control rlb_arm_control.launch use_leap:=true use_axis:=true connection:=tcpcoms"


exit 0
