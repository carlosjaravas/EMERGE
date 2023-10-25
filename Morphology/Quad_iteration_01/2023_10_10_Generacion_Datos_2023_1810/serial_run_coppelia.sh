#!/bin/bash
#$ -cwd
# sleep 50
cd /mnt/netapp2/Store_uni/home/ulc/ii/mnv
. ./bashrc2.sh
sleep 1
/mnt/netapp2/Store_uni/home/ulc/ii/mnv/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu18_04/coppeliaSim.sh -h -GzmqRemoteApi.rpcPort="$1" /mnt/netapp2/Home_FT2"$2"/quad_scene_v3_1.ttt & /mnt/netapp2/Home_FT2"$2"/serial_python_script.sh "$1" "$2"
