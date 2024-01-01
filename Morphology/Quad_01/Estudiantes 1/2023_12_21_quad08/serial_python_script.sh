#!/bin/bash
#$ -cwd
cd /mnt/netapp2/Home_FT2"$2"
sleep 50
python3 training_quad8.py "$1"
