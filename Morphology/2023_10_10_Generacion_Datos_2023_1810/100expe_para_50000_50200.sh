#!/bin/bash
#$ -cwd
for ((i = 50000; i <= 50200; i += 2)); do
  sbatch serial_run_baseScript.sh "$i"
  sleep 0.01
done
