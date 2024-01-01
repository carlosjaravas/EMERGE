#!/bin/sh
#SBATCH -c 1
#SBATCH --cpus-per-task=1
#SBATCH -t 01:00:00
#SBATCH --mem-per-cpu=3GB

#offscreen mode, headless mode compulsory
export QT_QPA_PLATFORM=offscreen
export LANG="en_US.UTF-8"

module load singularity

sleep 2

#under xvfb, X11 server available
#SINGULARITY_BIND="/mnt,/scratch" singularity exec $STORE/vrep.sif $HOME/V_REP_PRO_EDU_V3_6_2_Ubuntu18_04/xvfb-run_vrep-cesga.sh
ruta=$(pwd)
singularity exec -H $STORE/neat_rep $STORE/ubuntu-22.04-cesgapps.sif /mnt/netapp2/Home_FT2$ruta/serial_run_coppelia.sh "$1" $ruta
