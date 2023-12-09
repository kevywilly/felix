#!/bin/bash
cd ~/jetson-containers && ./build.sh --name=humble ros:humble-ros-base pytorch:1.10 pycuda cupy gstreamer torchvision
