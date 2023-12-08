#!/bin/bash
cd /nano-control && sudo /usr/bin/git pull
sudo /etc/init.d/nginx start
cd /felix