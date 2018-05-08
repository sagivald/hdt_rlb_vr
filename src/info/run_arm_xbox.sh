#!/bin/bash


terminator -e "roslaunch hdt_arm_control rlb_arm_control.launch use_axis:=true connection:=tcpcoms"


exit 0
