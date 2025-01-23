#!/bin/bash
clear
echo "Reaching bin directory..."
cd ~/flair/flair-install/bin/demos/armv7a-neon/SimpleControl
if [ $? -ne 0 ]; then
    echo "The bin path doesnt exist"
    exit 1
fi
scp -O CircleFollower_ardrone2.sh root@172.26.209.106:~/demos/AeroInes/
sleep 1s
scp -O CircleFollower_ardrone2.xml root@172.26.209.106:~/demos/AeroInes/
sleep 1s
scp -O SimpleControl_nrt root@172.26.209.106:~/demos/AeroInes/
if [ $? -ne 0 ]; then
    echo "Some of this files doesnt exist or its open somewhere XD"
    exit 1
fi
echo "Files uploaded"

