# LOAM_HESAI_PANDAR40P
A version that modifies LOAM algorithm to suit HESAI Pandar 40P Lidar
Author: Huang Pengdi
Email:alualu628628@163.com

This is a modification of LOAM algorithm
The code is to supprot Hesai Pandar 40 sensor while raw code is for velodyne Lidar sensor
Some related information is at https://github.com/HesaiTechnology/HesaiLidar-ros.git
One should be noted that only 16 laser beams have been selected to process LOAM algorithm

how to compile it?
the same as the raw LOAM algorithm, please see  https://github.com/laboshinl/loam_velodyne.git

how to use it?
roslaunch loam_velydone loam_hesai.launch
