#!/bin/bash
#$ -cwd
cd /mnt/netapp2/Home_FT2"$2"
sleep 50
python3 training_modular03.py "$1"
