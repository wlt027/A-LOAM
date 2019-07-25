# A-LOAM-zhkj
## Modified A-LOAM in support of zhkj devices

I made some modifies to support zhkj devices:  

1.use pangolin for 3d visualization(yeah,can not fall in love with rviz's 3d visualization)  
2.support dual laser scanner,check out aloam_zhkj_velodyne_dual.launch  
3.support other type laser scanner such as RS-16, pandar-40  
4.support save point cloud map data to disk 

Here is the original statement of A-LOAM:

A-LOAM is an Advanced implementation of LOAM (J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), which uses Eigen and Ceres Solver to simplify code structure. This code is modified from LOAM and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED). This code is clean and simple without complicated mathematical derivation and redundant operations. It is a good learning material for SLAM beginners.

<img src="https://github.com/wlt027/A-LOAM/picture/M-Aloam.png" width = 55% height = 55%/>

**Modifier:**[Chi Wei](https://github.com/wlt027/A-LOAM), [Tong Qin](http://www.qintonguav.com), [Shaozu Cao](https://github.com/shaozu)


## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).


## 2. Build A-LOAM
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/A-LOAM.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Velodyne VLP-16 Example
Download [NSH indoor outdoor](https://drive.google.com/file/d/1s05tBQOLNEDDurlg48KiUWxCp-YqYyGH/view) to YOUR_DATASET_FOLDER. 

```
    roslaunch aloam_velodyne aloam_velodyne_VLP_16.launch
    rosbag play YOUR_DATASET_FOLDER/nsh_indoor_outdoor.bag
```


## 4. Dual-Velodyne VLP-16 Example
Move zhkj's data to YOUR_DATASET_FOLDER. 

```
    roslaunch aloam_velodyne aloam_zhkj_velodyne_dual.launch
    rosbag play YOUR_DATASET_FOLDER/zhkj_indoor.bag
```


## 5. KITTI Example (Velodyne HDL-64)
Download [KITTI Odometry dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) to YOUR_DATASET_FOLDER and set the `dataset_folder` and `sequence_number` parameters in `kitti_helper.launch` file. Note you also convert KITTI dataset to bag file for easy use by setting proper parameters in `kitti_helper.launch`. 

```
    roslaunch aloam_velodyne aloam_velodyne_HDL_64.launch
    roslaunch aloam_velodyne kitti_helper.launch
```
<img src="https://github.com/HKUST-Aerial-Robotics/A-LOAM/blob/devel/picture/kitti_gif.gif" width = 720 height = 351 />

## 6. Docker Support
To further facilitate the building process, we add docker in our code. Docker environment is like a sandbox, thus makes our code environment-independent. To run with docker, first make sure [ros](http://wiki.ros.org/ROS/Installation) and [docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/) are installed on your machine. Then add your account to `docker` group by `sudo usermod -aG docker $YOUR_USER_NAME`. **Relaunch the terminal or logout and re-login if you get `Permission denied` error**, type:
```
cd ~/catkin_ws/src/A-LOAM/docker
make build
```
The build process may take a while depends on your machine. After that, run `./run.sh 16` or `./run.sh 64` to launch A-LOAM, then you should be able to see the result.


## 7.Acknowledgements
Thanks for [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM), LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time) and [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).

