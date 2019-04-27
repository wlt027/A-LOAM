// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#define PCL_NO_PRECOMPILE

#include <cmath>
#include <vector>
#include <string>
#include <tuple>
#include <queue>
#include <condition_variable>
#include <mutex>
#include <thread>
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "aloam_velodyne/voxel_grid_keep_property.h"
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>

using std::atan2;
using std::cos;
using std::sin;

const double scanPeriod = 0.1;

const int systemDelay = 0; 
int systemInitCount = 0;
bool systemInited = false;
int num_of_lidar = 1;
int N_SCANS0 = 0;
std::string LIDAR0_TOPIC = "/velodyne_points";
int N_SCANS1= 0;
std::string LIDAR1_TOPIC = "/velodyne_points";

Eigen::Vector3f PBL0(0,0,0);
Eigen::Quaternionf QBL0(1,0,0,0);
Eigen::Vector3f PBL1(0,0,0);
Eigen::Quaternionf QBL1(1,0,0,0);

float cloudCurvature0[400000];
int cloudSortInd0[400000];
int cloudNeighborPicked0[400000];
int cloudLabel0[400000];

float cloudCurvature1[400000];
int cloudSortInd1[400000];
int cloudNeighborPicked1[400000];
int cloudLabel1[400000];

bool comp0 (int i,int j) { return (cloudCurvature0[i]<cloudCurvature0[j]); }
bool comp1 (int i,int j) { return (cloudCurvature1[i]<cloudCurvature1[j]); }

typedef std::tuple<double,pcl::PointCloud<PointType>,pcl::PointCloud<PointType>, pcl::PointCloud<PointType>,pcl::PointCloud<PointType>,pcl::PointCloud<PointType>> LoamFeatures;

std::queue<LoamFeatures> laser0Buf;
std::queue<LoamFeatures> laser1Buf;
std::mutex m_buf;
std::condition_variable con;

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;

double MINIMUM_RANGE0 = 0.1;

template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out, float thres)
{
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.points.size());
    }

    size_t j = 0;

    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres * thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    if (!systemInited)
    { 
        systemInitCount++;
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        else
            return;
    }

    TicToc t_whole;
    TicToc t_prepare;
    std::vector<int> scanStartInd(N_SCANS0, 0);
    std::vector<int> scanEndInd(N_SCANS0, 0);

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE0);

    int cloudSize = laserCloudIn.points.size();
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;

    if(N_SCANS0 == 40)
    {
        startOri = -atan2(laserCloudIn.points[0].x, -laserCloudIn.points[0].y);
        endOri = -atan2(laserCloudIn.points[cloudSize - 1].x,
                              -laserCloudIn.points[cloudSize - 1].y) +
                       2 * M_PI;
    }

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    //printf("end Ori %f\n", endOri);

    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS0);
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        if(N_SCANS0 == 40)
        {
            point.x = -laserCloudIn.points[i].y;
            point.y = laserCloudIn.points[i].x;
            point.z = laserCloudIn.points[i].z;
        }

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if (N_SCANS0 == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS0 - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS0 == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS0 - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS0 == 40)
        {
            // pandar 40 vertical angle is un-uniform
            if(angle > -6 && angle < 2)
            {
                scanID = int((angle + 6) * 0.33 + 0.5) + 10;
            } else if(angle <= -6)
            {
                scanID = int((angle + 16) * 1 + 0.5);
            }
            else
            {
                scanID = int((angle - 2) * 1 + 0.5) + 35;
            }

            if (scanID > (N_SCANS0 - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS0 == 64)
        {
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS0 / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies 
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);

        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        { 
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - startOri) / (endOri - startOri);
        point.intensity = scanID + scanPeriod * relTime;

        Eigen::Vector3f coord_l(point.x,point.y,point.z);
        Eigen::Vector3f coord_b = QBL0*coord_l + PBL0;
        PointType point_b;
        point_b.x = coord_b(0);
        point_b.y = coord_b(1);
        point_b.z = coord_b(2);
        point_b.intensity = point.intensity;
        laserCloudScans[scanID].push_back(point_b);
    }
    
    cloudSize = count;
    printf("points size %d \n", cloudSize);

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < N_SCANS0; i++)
    { 
        scanStartInd[i] = laserCloud->size() + 5;
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;
    }

    printf("prepare time %f \n", t_prepare.toc());

    for (int i = 5; i < cloudSize - 5; i++)
    { 
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        cloudCurvature0[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd0[i] = i;
        cloudNeighborPicked0[i] = 0;
        cloudLabel0[i] = 0;
    }

    TicToc t_pts;

    pcl::PointCloud<PointType> cornerPointsSharp;
    pcl::PointCloud<PointType> cornerPointsLessSharp;
    pcl::PointCloud<PointType> surfPointsFlat;
    pcl::PointCloud<PointType> surfPointsLessFlat;

    float t_q_sort = 0;
    for (int i = 0; i < N_SCANS0; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
        for (int j = 0; j < 6; j++)
        {
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            TicToc t_tmp;
            std::sort (cloudSortInd0 + sp, cloudSortInd0 + ep + 1, comp0);
            t_q_sort += t_tmp.toc();

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSortInd0[k];

                if (cloudNeighborPicked0[ind] == 0 &&
                    cloudCurvature0[ind] > 0.1)
                {

                    largestPickedNum++;
                    if (largestPickedNum <= 2)
                    {                        
                        cloudLabel0[ind] = 2;
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else if (largestPickedNum <= 20)
                    {                        
                        cloudLabel0[ind] = 1;
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    cloudNeighborPicked0[ind] = 1;

                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked0[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked0[ind + l] = 1;
                    }
                }
            }

            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd0[k];

                if (cloudNeighborPicked0[ind] == 0 &&
                    cloudCurvature0[ind] < 0.1)
                {

                    cloudLabel0[ind] = -1;
                    surfPointsFlat.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    { 
                        break;
                    }

                    cloudNeighborPicked0[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    { 
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked0[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked0[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel0[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<PointType> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
    printf("sort q time %f \n", t_q_sort);
    printf("seperate points time %f \n", t_pts.toc());


    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "/camera_init";
    if(num_of_lidar == 1)
        pubLaserCloud.publish(laserCloudOutMsg);

    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "/camera_init";
    if(num_of_lidar == 1)
        pubCornerPointsSharp.publish(cornerPointsSharpMsg);

    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "/camera_init";
    if(num_of_lidar == 1)
        pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "/camera_init";
    if(num_of_lidar == 1)
        pubSurfPointsFlat.publish(surfPointsFlat2);

    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "/camera_init";
    if(num_of_lidar == 1)
        pubSurfPointsLessFlat.publish(surfPointsLessFlat2);

    if(num_of_lidar == 2)
    {
        m_buf.lock();
        laser0Buf.push(std::make_tuple(laserCloudMsg->header.stamp.toSec(),*laserCloud,cornerPointsSharp,cornerPointsLessSharp,surfPointsFlat,surfPointsLessFlat));
        m_buf.unlock();
        con.notify_one();
    }

    printf("scan registration time %f ms *************\n", t_whole.toc());
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");
}

std::vector<LoamFeatures> getMeasurementsDual()
{
    std::vector<LoamFeatures> measurements;

    while (true)
    {
        // make sure data buffer have data
        if (laser0Buf.empty() || laser1Buf.empty())
            return measurements;

        // we use laser0 time as base
        if (!(std::get<0>(laser1Buf.back())  > std::get<0>(laser0Buf.front())))
        {
            //printf("wait for imu, only should happen at the beginning\n");
            return measurements;
        }

        if (!(std::get<0>(laser1Buf.front()) < std::get<0>(laser0Buf.front())))
        {
            printf("throw laser0 , only should happen at the beginning\n");
            laser0Buf.pop();
            continue;
        }

        // get laser0 measurement
        double time = std::get<0>(laser0Buf.front());
        pcl::PointCloud<PointType> fulllaser;
        pcl::copyPointCloud(std::get<1>(laser0Buf.front()),fulllaser);
        pcl::PointCloud<PointType> corners,surfels,lessCorners,lessSurfels;
        pcl::copyPointCloud(std::get<2>(laser0Buf.front()),corners);
        pcl::copyPointCloud(std::get<3>(laser0Buf.front()),surfels);
        pcl::copyPointCloud(std::get<4>(laser0Buf.front()),lessCorners);
        pcl::copyPointCloud(std::get<5>(laser0Buf.front()),lessSurfels);
        laser0Buf.pop();

        // get laser1 measurement
        while (std::get<0>(laser1Buf.front()) < time)
        {
            pcl::PointCloud<PointType> tmp_corners,tmp_surfels,tmp_lessCorners,tmp_lessSurfels;
            pcl::copyPointCloud(std::get<2>(laser1Buf.front()),tmp_corners);
            pcl::copyPointCloud(std::get<3>(laser1Buf.front()),tmp_surfels);
            pcl::copyPointCloud(std::get<4>(laser1Buf.front()),tmp_lessCorners);
            pcl::copyPointCloud(std::get<5>(laser1Buf.front()),tmp_lessSurfels);

            corners += tmp_corners;
            surfels += tmp_surfels;
            lessCorners += tmp_lessCorners;
            lessSurfels += tmp_lessSurfels;

            laser1Buf.pop();
        }

//        {
//            // output
//            std::string strOut = "/media/wlt/DATA/data/zhkj/190425/";
//            double scanTime = time;
//
//            {
//                std::string strCorner = strOut + std::to_string(scanTime) + "-corner.txt";
//
//                FILE* pFile = fopen(strCorner.data(),"w");
//
//                for(int i = 0; i< corners.size();i++)
//                {
//                    PointType pt = corners[i];
//
//                    fprintf(pFile,"%f,%f,%f,%f\n",pt.x,pt.y,pt.z,pt.intensity);
//                }
//                fclose(pFile);
//            }
//
//            {
//                std::string strCorner = strOut + std::to_string(scanTime) + "-cornerLess.txt";
//
//                FILE* pFile = fopen(strCorner.data(),"w");
//
//                for(int i = 0; i< lessCorners.size();i++)
//                {
//                    PointType pt = lessCorners[i];
//
//                    fprintf(pFile,"%f,%f,%f,%f\n",pt.x,pt.y,pt.z,pt.intensity);
//                }
//                fclose(pFile);
//            }
//
//            {
//                std::string strCorner = strOut + std::to_string(scanTime) + "-surfel.txt";
//
//                FILE* pFile = fopen(strCorner.data(),"w");
//
//                for(int i = 0; i< surfels.size();i++)
//                {
//                    PointType pt = surfels[i];
//
//                    fprintf(pFile,"%%f,%f,%f,%f\n",pt.x,pt.y,pt.z,pt.intensity);
//                }
//                fclose(pFile);
//            }
//
//            {
//                std::string strCorner = strOut + std::to_string(scanTime) + "-surfelLess.txt";
//
//                FILE* pFile = fopen(strCorner.data(),"w");
//
//                for(int i = 0; i< lessSurfels.size();i++)
//                {
//                    PointType pt = lessSurfels[i];
//
//                    fprintf(pFile,"%f,%f,%f,%f\n",pt.x,pt.y,pt.z,pt.intensity);
//                }
//                fclose(pFile);
//            }
//
//        }
        measurements.emplace_back(std::make_tuple(time, fulllaser,corners,surfels,lessCorners,lessSurfels));
    }
    return measurements;
}

void process()
{
    if(num_of_lidar == 1)
        return;

    while (1) {
        std::vector<LoamFeatures> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&] {
                return (measurements = getMeasurementsDual()).size() != 0;
        });
        lk.unlock();

        for (auto &measurement : measurements) {

            double time = std::get<0>(measurement);

            sensor_msgs::PointCloud2 laserCloudOutMsg;
            pcl::toROSMsg(std::get<1>(measurement), laserCloudOutMsg);
            laserCloudOutMsg.header.stamp.fromSec(time);
            laserCloudOutMsg.header.frame_id = "/camera_init";
            pubLaserCloud.publish(laserCloudOutMsg);

            sensor_msgs::PointCloud2 cornerPointsSharpMsg;
            pcl::toROSMsg(std::get<2>(measurement), cornerPointsSharpMsg);
            cornerPointsSharpMsg.header.stamp.fromSec(time);
            cornerPointsSharpMsg.header.frame_id = "/camera_init";
            pubCornerPointsSharp.publish(cornerPointsSharpMsg);

            sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
            pcl::toROSMsg(std::get<3>(measurement), cornerPointsLessSharpMsg);
            cornerPointsLessSharpMsg.header.stamp.fromSec(time);
            cornerPointsLessSharpMsg.header.frame_id = "/camera_init";
            pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

            sensor_msgs::PointCloud2 surfPointsFlat2;
            pcl::toROSMsg(std::get<4>(measurement), surfPointsFlat2);
            surfPointsFlat2.header.stamp.fromSec(time);
            surfPointsFlat2.header.frame_id = "/camera_init";
            pubSurfPointsFlat.publish(surfPointsFlat2);

            sensor_msgs::PointCloud2 surfPointsLessFlat2;
            pcl::toROSMsg(std::get<5>(measurement), surfPointsLessFlat2);
            surfPointsLessFlat2.header.stamp.fromSec(time);
            surfPointsLessFlat2.header.frame_id = "/camera_init";
            pubSurfPointsLessFlat.publish(surfPointsLessFlat2);
        }
    }
}

void laserCloud1Handler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    if (!systemInited)
    {
        systemInitCount++;
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        else
            return;
    }

    TicToc t_whole;
    TicToc t_prepare;
    std::vector<int> scanStartInd(N_SCANS1, 0);
    std::vector<int> scanEndInd(N_SCANS1, 0);

    pcl::PointCloud<PointXYZIRTRaw> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    std::vector<int> indices;

    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    removeClosedPointCloud(laserCloudIn, laserCloudIn, MINIMUM_RANGE0);

    int cloudSize = laserCloudIn.points.size();
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;

    if(N_SCANS1 == 40)
    {
        startOri = -atan2(laserCloudIn.points[0].x, -laserCloudIn.points[0].y);
        endOri = -atan2(laserCloudIn.points[cloudSize - 1].x,
                        -laserCloudIn.points[cloudSize - 1].y) +
                 2 * M_PI;
    }

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    //printf("end Ori %f\n", endOri);

    bool halfPassed = false;
    int count = cloudSize;
    PointType2 point;
    std::vector<pcl::PointCloud<PointType2>> laserCloudScans(N_SCANS1);
    for (int i = 0; i < cloudSize; i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        point.time = laserCloudIn.points[i].timestamp*1e-09;

        if(N_SCANS1 == 40)
        {
            point.x = -laserCloudIn.points[i].y;
            point.y = laserCloudIn.points[i].x;
            point.z = laserCloudIn.points[i].z;
            point.time = laserCloudIn.points[i].timestamp*1e-09;
        }

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID = 0;

        if (N_SCANS1 == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS1 - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS1 == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS1 - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS1 == 40)
        {
            // pandar 40 vertical angle is un-uniform
            if(angle > -6 && angle < 2)
            {
                scanID = int((angle + 6) * 0.33 + 0.5) + 10;
            } else if(angle <= -6)
            {
                scanID = int((angle + 16) * 1 + 0.5);
            }
            else
            {
                scanID = int((angle - 2) * 1 + 0.5) + 35;
            }

            if (scanID > (N_SCANS1 - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else if (N_SCANS1 == 64)
        {
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS1 / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies
            if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
            ROS_BREAK();
        }
        //printf("angle %f scanID %d \n", angle, scanID);

        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        {
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        float relTime = (ori - startOri) / (endOri - startOri);
        point.intensity = scanID + scanPeriod * relTime;

        Eigen::Vector3f coord_l(point.x,point.y,point.z);
        Eigen::Vector3f coord_b = QBL1*coord_l + PBL1;
        PointType2 point_b;
        point_b.time = point.time;
        point_b.x = coord_b(0);
        point_b.y = coord_b(1);
        point_b.z = coord_b(2);
        point_b.intensity = point.intensity;
        laserCloudScans[scanID].push_back(point_b);
    }

    cloudSize = count;
    printf("points size %d \n", cloudSize);

    pcl::PointCloud<PointType2>::Ptr laserCloud(new pcl::PointCloud<PointType2>());
    for (int i = 0; i < N_SCANS1; i++)
    {
        scanStartInd[i] = laserCloud->size() + 5;
        *laserCloud += laserCloudScans[i];
        scanEndInd[i] = laserCloud->size() - 6;
    }

    printf("prepare time %f \n", t_prepare.toc());

    for (int i = 5; i < cloudSize - 5; i++)
    {
        float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
        float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
        float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;

        cloudCurvature1[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloudSortInd1[i] = i;
        cloudNeighborPicked1[i] = 0;
        cloudLabel1[i] = 0;
    }


    TicToc t_pts;

    pcl::PointCloud<PointType2> cornerPointsSharp;
    pcl::PointCloud<PointType2> cornerPointsLessSharp;
    pcl::PointCloud<PointType2> surfPointsFlat;
    pcl::PointCloud<PointType2> surfPointsLessFlat;

    float t_q_sort = 0;
    for (int i = 0; i < N_SCANS1; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        pcl::PointCloud<PointType2>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType2>);
        for (int j = 0; j < 6; j++)
        {
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            TicToc t_tmp;
            std::sort (cloudSortInd1 + sp, cloudSortInd1 + ep + 1, comp1);
            t_q_sort += t_tmp.toc();

            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSortInd1[k];

                if (cloudNeighborPicked1[ind] == 0 &&
                    cloudCurvature1[ind] > 0.1)
                {

                    largestPickedNum++;
                    if (largestPickedNum <= 2)
                    {
                        cloudLabel1[ind] = 2;
                        cornerPointsSharp.push_back(laserCloud->points[ind]);
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else if (largestPickedNum <= 20)
                    {
                        cloudLabel1[ind] = 1;
                        cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    cloudNeighborPicked1[ind] = 1;

                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked1[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked1[ind + l] = 1;
                    }
                }
            }

            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd1[k];

                if (cloudNeighborPicked1[ind] == 0 &&
                    cloudCurvature1[ind] < 0.1)
                {

                    cloudLabel1[ind] = -1;
                    surfPointsFlat.push_back(laserCloud->points[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 4)
                    {
                        break;
                    }

                    cloudNeighborPicked1[ind] = 1;
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked1[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                        float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                        float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked1[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel1[k] <= 0)
                {
                    surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                }
            }
        }

        pcl::PointCloud<PointType2> surfPointsLessFlatScanDS;
        pcl::VoxelGridKeepFields<PointType2> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
    printf("sort q time %f \n", t_q_sort);
    printf("seperate points time %f \n", t_pts.toc());

    // spin laser feature to 10 bin
    {
        double scanTime = laserCloudMsg->header.stamp.toSec();
        int m_binCount = 10;
        double m_binTimestep = scanPeriod / (float) m_binCount;

        pcl::PointCloud<PointType> fulllaser;
        std::vector<pcl::PointCloud<PointType>> vecCornersBins;
        std::vector<pcl::PointCloud<PointType>> vecSurfelsBins;
        std::vector<pcl::PointCloud<PointType>> vecLessCornersBins;
        std::vector<pcl::PointCloud<PointType>> vecLessSufelsBins;

        for(int i = 0; i< m_binCount;i++)
        {
            pcl::PointCloud<PointType> pcd0;
            vecCornersBins.push_back(pcd0);
            pcl::PointCloud<PointType> pcd1;
            vecSurfelsBins.push_back(pcd1);
            pcl::PointCloud<PointType> pcd2;
            vecLessCornersBins.push_back(pcd2);
            pcl::PointCloud<PointType> pcd3;
            vecLessSufelsBins.push_back(pcd3);
        }

        for (int idx = 0; idx < cornerPointsSharp.size(); idx++) {
            PointType2 pt = cornerPointsSharp[idx];

            double pt_time = pt.time;

            if (scanTime - pt_time < 0) {
                printf("time exception: %lf\n", pt_time);
            }
            int nID = (scanTime - pt_time) / m_binTimestep;
            nID = m_binCount - 1 - nID;

            if (nID == -1) {
                nID = 0;
            }
            if (nID < 0 || nID > m_binCount - 1) {
                printf("id exception: %d\n", nID);
                printf("info: %lf,%lf,%lf\n", scanTime, pt_time, m_binTimestep);
                continue;
            }

            if (pcl_isnan(pt.x) || pcl_isnan(pt.y) || pcl_isnan(pt.z))
                continue;
            PointType p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            vecCornersBins[nID].push_back(p);
        }

        for (int idx = 0; idx < cornerPointsLessSharp.size(); idx++) {
            PointType2 pt = cornerPointsLessSharp[idx];

            double pt_time = pt.time;

            if (scanTime - pt_time < 0) {
                printf("time exception: %lf\n", pt_time);
            }
            int nID = (scanTime - pt_time) / m_binTimestep;
            nID = m_binCount - 1 - nID;

            if (nID == -1) {
                nID = 0;
            }
            if (nID < 0 || nID > m_binCount - 1) {
                printf("id exception: %d\n", nID);
                printf("info: %lf,%lf,%lf\n", scanTime, pt_time, m_binTimestep);
                continue;
            }

            if (pcl_isnan(pt.x) || pcl_isnan(pt.y) || pcl_isnan(pt.z))
                continue;

            PointType p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;

            vecLessCornersBins[nID].push_back(p);
        }

        for (int idx = 0; idx < surfPointsFlat.size(); idx++) {
            PointType2 pt = surfPointsFlat[idx];

            double pt_time = pt.time;

            if (scanTime - pt_time < 0) {
                printf("time exception: %lf\n", pt_time);
            }
            int nID = (scanTime - pt_time) / m_binTimestep;
            nID = m_binCount - 1 - nID;

            if (nID == -1) {
                nID = 0;
            }
            if (nID < 0 || nID > m_binCount - 1) {
                printf("id exception: %d\n", nID);
                printf("info: %lf,%lf,%lf\n", scanTime, pt_time, m_binTimestep);
                continue;
            }

            if (pcl_isnan(pt.x) || pcl_isnan(pt.y) || pcl_isnan(pt.z))
                continue;

            PointType p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;

            vecLessCornersBins[nID].push_back(p);
        }

        for (int idx = 0; idx < surfPointsLessFlat.size(); idx++) {
            PointType2 pt = surfPointsLessFlat[idx];

            double pt_time = pt.time;

            if (scanTime - pt_time < 0) {
                printf("time exception: %lf\n", pt_time);
            }
            int nID = (scanTime - pt_time) / m_binTimestep;
            nID = m_binCount - 1 - nID;

            if (nID == -1) {
                nID = 0;
            }
            if (nID < 0 || nID > m_binCount - 1) {
                printf("id exception: %d\n", nID);
                printf("info: %lf,%lf,%lf\n", scanTime, pt_time, m_binTimestep);
                continue;
            }

            if (pcl_isnan(pt.x) || pcl_isnan(pt.y) || pcl_isnan(pt.z))
                continue;

            PointType p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;

            vecLessSufelsBins[nID].push_back(p);
        }

        for(int i = 0; i< m_binCount;i++)
        {
            pcl::PointCloud<PointType> pcd0 = vecCornersBins[i];
            pcl::PointCloud<PointType> pcd1 = vecSurfelsBins[i];
            pcl::PointCloud<PointType> pcd2 = vecLessCornersBins[i];
            pcl::PointCloud<PointType> pcd3 = vecLessSufelsBins[i];
            double binTime = scanTime - (m_binCount - 1 - i)*m_binTimestep;

            m_buf.lock();
            laser1Buf.push(std::make_tuple(binTime,fulllaser,pcd0,pcd1,pcd2,pcd3));
            m_buf.unlock();
            con.notify_one();
        }
    }

    printf("scan registration time %f ms *************\n", t_whole.toc());
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanRegistration");
    ros::NodeHandle nh;

    nh.param<int>("lidar_num",num_of_lidar,1);
    nh.param<double>("minimum_range", MINIMUM_RANGE0, 0.1);

    ros::Subscriber subLaserCloud0,subLaserCloud1;

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    if(num_of_lidar == 1)
    {
        nh.param<int>("scan_line", N_SCANS0, 16);
        nh.getParam("lidar_topic", LIDAR0_TOPIC);
        printf("scan line number %d \n", N_SCANS0);
        printf("lidar topic:%s \n",LIDAR0_TOPIC.data());

        if(N_SCANS0 != 16 && N_SCANS0 != 32 && N_SCANS0 != 64 && N_SCANS0 != 40)
        {
            printf("only support velodyne with 16, 32 or 64 scan line,robosense16, Hesai Pandar 40!");
            return 0;
        }

        subLaserCloud0 = nh.subscribe<sensor_msgs::PointCloud2>(LIDAR0_TOPIC, 100, laserCloudHandler);

    } else if(num_of_lidar == 2)
    {
        nh.param<int>("scan0_line", N_SCANS0, 16);
        nh.getParam("lidar0_topic", LIDAR0_TOPIC);
        printf("scan line number %d \n", N_SCANS0);
        printf("lidar topic:%s \n",LIDAR0_TOPIC.data());

        {
            float x,y,z,rx,ry,rz;
            nh.param<float>("lidar0_x",x,0);
            nh.param<float>("lidar0_y",y,0);
            nh.param<float>("lidar0_z",z,0);
            nh.param<float>("lidar0_rx",rx,0);
            nh.param<float>("lidar0_ry",ry,0);
            nh.param<float>("lidar0_rz",rz,0);

            PBL0 = Eigen::Vector3f(x,y,z);
            QBL0 = Eigen::Quaternionf(Eigen::AngleAxisf(rz * M_PI / 180.0, Eigen::Vector3f::UnitZ())
                                      * Eigen::AngleAxisf(ry * M_PI / 180.0, Eigen::Vector3f::UnitY())
                                      * Eigen::AngleAxisf(rx * M_PI / 180.0, Eigen::Vector3f::UnitX()));
        }

        if(N_SCANS0 != 16 && N_SCANS0 != 32 && N_SCANS0 != 64 && N_SCANS0 != 40)
        {
            printf("only support velodyne with 16, 32 or 64 scan line,robosense16, Hesai Pandar 40!");
            return 0;
        }
        subLaserCloud0 = nh.subscribe<sensor_msgs::PointCloud2>(LIDAR0_TOPIC, 100, laserCloudHandler);

        nh.param<int>("scan1_line", N_SCANS1, 16);
        nh.getParam("lidar1_topic", LIDAR1_TOPIC);
        printf("scan line number %d \n", N_SCANS1);
        printf("lidar topic:%s \n",LIDAR1_TOPIC.data());

        {
            float x,y,z,rx,ry,rz;
            nh.param<float>("lidar1_x",x,0);
            nh.param<float>("lidar1_y",y,0);
            nh.param<float>("lidar1_z",z,0);
            nh.param<float>("lidar1_rx",rx,0);
            nh.param<float>("lidar1_ry",ry,0);
            nh.param<float>("lidar1_rz",rz,0);

            PBL1 = Eigen::Vector3f(x,y,z);
            QBL1 = Eigen::Quaternionf(Eigen::AngleAxisf(rz * M_PI / 180.0, Eigen::Vector3f::UnitZ())
                                      * Eigen::AngleAxisf(ry * M_PI / 180.0, Eigen::Vector3f::UnitY())
                                      * Eigen::AngleAxisf(rx * M_PI / 180.0, Eigen::Vector3f::UnitX()));
        }

        bool bSupport = true;
        if(N_SCANS1 != 16 && N_SCANS1 != 32 && N_SCANS1 != 64 && N_SCANS1 != 40)
        {
            printf("only support velodyne with 16, 32 or 64 scan line,robosense16, Hesai Pandar 40!");
            bSupport = false;
        }

        if (bSupport) {
            subLaserCloud1 = nh.subscribe<sensor_msgs::PointCloud2>(LIDAR1_TOPIC, 100, laserCloud1Handler);

        } else{
            // only support single laser
            num_of_lidar = 1;
        }
    }

    std::thread syn_process{process};

    ros::spin();

    return 0;
}
