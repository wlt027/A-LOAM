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

#include <math.h>
#include <vector>
#include <aloam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>

#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <tuple>

#include "lidarFactor.hpp"
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"

#include "aloam_velodyne/CircularBuffer.h"
#include <fstream>

int frameCount = 0;
int keyscanID = 0;
int savefreq = 5;
int rmapOut = 0;
int nGPS = 0;

double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometry = 0;

int laserCloudCenWidth = 10;
int laserCloudCenHeight = 10;
int laserCloudCenDepth = 5;
const int laserCloudWidth = 21;
const int laserCloudHeight = 21;
const int laserCloudDepth = 11;

const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; //4851

int laserCloudValidInd[125];
int laserCloudSurroundInd[125];

class GPSReading {
public:

    enum COV_TYPE {
        COVARIANCE_TYPE_UNKNOWN = 0u,
        COVARIANCE_TYPE_APPROXIMATED = 1u,
        COVARIANCE_TYPE_DIAGONAL_KNOWN = 2u,
        COVARIANCE_TYPE_KNOWN = 3u,
    };

    enum FIX_TYPE {
        STATUS_NO_FIX = -1,
        STATUS_FIX = 0,
        STATUS_SBAS_FIX = 1,
        STATUS_GBAS_FIX = 2
    };

    double timestamp;
    Eigen::Vector3d lla;
    Eigen::Matrix3d covariance;
    COV_TYPE cov_type;
    FIX_TYPE fix_type;
    double heading;
    double headingCov;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// input: from odom
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());

// ouput: all visualble cube points
pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());

int nSubmapBinSize = 0;
double keyframe_dis = 1;

//std::queue<pcl::PointCloud<pcl::PointXYZI>> submap;
alive::CircularBuffer<pcl::PointCloud<pcl::PointXYZI>> submap{10};

// surround points in map to build tree
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());

//input & output: points in one frame. local --> global
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());

// points in every cube
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];

//kd-tree
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4);

Eigen::Vector3d t_w_pre(0, 0, 0);

// wmap_T_odom * odom_T_curr = wmap_T_curr;
// transformation between odom's world and map's world frame
Eigen::Quaterniond q_wmap_wodom(1, 0, 0, 0);
Eigen::Vector3d t_wmap_wodom(0, 0, 0);

Eigen::Quaterniond q_wodom_curr(1, 0, 0, 0);
Eigen::Vector3d t_wodom_curr(0, 0, 0);


std::queue<sensor_msgs::PointCloud2ConstPtr> cornerLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> surfLastBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::queue<GPSReading> gpsBuf;

std::queue<std::tuple<int,int,double,pcl::PointCloud<pcl::PointXYZI>,pcl::PointCloud<pcl::PointXYZI>,pcl::PointCloud<pcl::PointXYZI>,Eigen::Vector3d,Eigen::Quaterniond,std::vector<GPSReading>>> key_scan_buf;

std::mutex mBuf;
std::mutex mSave;

pcl::VoxelGrid<PointType> downSizeFilterCorner;
pcl::VoxelGrid<PointType> downSizeFilterSurf;

std::vector<int> pointSearchInd;
std::vector<float> pointSearchSqDis;

PointType pointOri, pointSel;

ros::Publisher pubLaserCloudSurround, pubLaserCloudMap, pubLaserCloudFullRes, pubOdomAftMapped, pubOdomAftMappedHighFrec, pubLaserAfterMappedPath;

nav_msgs::Path laserAfterMappedPath;

std::string strOut;

std::ofstream ofs;

// set initial guess
void transformAssociateToMap()
{
	q_w_curr = q_wmap_wodom * q_wodom_curr;
	t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
}

void transformUpdate()
{
	q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
	t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
}

void pointAssociateToMap(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
	po->x = point_w.x();
	po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
	//po->intensity = 1.0;
}

void pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
	po->x = point_curr.x();
	po->y = point_curr.y();
	po->z = point_curr.z();
	po->intensity = pi->intensity;
}

void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2)
{
	mBuf.lock();
	cornerLastBuf.push(laserCloudCornerLast2);
	mBuf.unlock();
}

void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2)
{
	mBuf.lock();
	surfLastBuf.push(laserCloudSurfLast2);
	mBuf.unlock();
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
	mBuf.lock();
	fullResBuf.push(laserCloudFullRes2);
	mBuf.unlock();
}

//receive odomtry
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
	mBuf.lock();
	odometryBuf.push(laserOdometry);
	mBuf.unlock();

	// high frequence publish
	Eigen::Quaterniond q_wodom_curr;
	Eigen::Vector3d t_wodom_curr;
	q_wodom_curr.x() = laserOdometry->pose.pose.orientation.x;
	q_wodom_curr.y() = laserOdometry->pose.pose.orientation.y;
	q_wodom_curr.z() = laserOdometry->pose.pose.orientation.z;
	q_wodom_curr.w() = laserOdometry->pose.pose.orientation.w;
	t_wodom_curr.x() = laserOdometry->pose.pose.position.x;
	t_wodom_curr.y() = laserOdometry->pose.pose.position.y;
	t_wodom_curr.z() = laserOdometry->pose.pose.position.z;

	Eigen::Quaterniond q_w_curr = q_wmap_wodom * q_wodom_curr;
	Eigen::Vector3d t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom; 

	nav_msgs::Odometry odomAftMapped;
	odomAftMapped.header.frame_id = "/camera_init";
	odomAftMapped.child_frame_id = "/aft_mapped";
	odomAftMapped.header.stamp = laserOdometry->header.stamp;
	odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
	odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
	odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
	odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
	odomAftMapped.pose.pose.position.x = t_w_curr.x();
	odomAftMapped.pose.pose.position.y = t_w_curr.y();
	odomAftMapped.pose.pose.position.z = t_w_curr.z();
	pubOdomAftMappedHighFrec.publish(odomAftMapped);
}

void outputRMap(const int &scanId, const int &keyScanId, const double &time,
                const pcl::PointCloud<pcl::PointXYZI> &fullRect, const pcl::PointCloud<pcl::PointXYZI> &lessSharpRect, const pcl::PointCloud<pcl::PointXYZI> &lessFlatRect,
                const Eigen::Vector3d& translation,const Eigen::Quaterniond& quaternion,std::vector<GPSReading>& gpsVec) {
    //ID
    long long scanID = scanId;
    long long keyScanID = keyScanId;
    double scanTimestamp = time;

    ofs.write(reinterpret_cast<const char*>(&scanID), sizeof(long long));
    ofs.write(reinterpret_cast<const char*>(&keyScanID), sizeof(long long));
    ofs.write(reinterpret_cast<const char*>(&scanTimestamp), sizeof(double));

    //SubMap
    int sizeOfSubMap = 0;
    ofs.write(reinterpret_cast<const char*>(&sizeOfSubMap), sizeof(int));

    //laserScanRect
    int sizeOfRawScanRect = fullRect.points.size();
    ofs.write(reinterpret_cast<const char*>(&sizeOfRawScanRect), sizeof(int));
    for (auto p : fullRect.points) {
        float x = p.x;
        float y = p.y;
        float z = p.z;
        uint8_t intensity = p.intensity;
        ofs.write(reinterpret_cast<const char*>(&x), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&y), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&z), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&intensity), sizeof(uint8_t));
    }

    //laserScanCornerRect
    int sizeOfScanCornerRect = lessSharpRect.points.size();
    ofs.write(reinterpret_cast<const char*>(&sizeOfScanCornerRect), sizeof(int));
    for (auto p : lessSharpRect.points) {
        float x = p.x;
        float y = p.y;
        float z = p.z;
        uint8_t intensity = p.intensity;
        ofs.write(reinterpret_cast<const char*>(&x), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&y), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&z), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&intensity), sizeof(uint8_t));
    }

    //laserScanSurfRect
    int sizeOfScanSurfRect = lessFlatRect.points.size();
    ofs.write(reinterpret_cast<const char*>(&sizeOfScanSurfRect), sizeof(int));
    for (auto p : lessFlatRect.points) {
        float x = p.x;
        float y = p.y;
        float z = p.z;
        uint8_t intensity = p.intensity;
        ofs.write(reinterpret_cast<const char*>(&x), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&y), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&z), sizeof(float));
        ofs.write(reinterpret_cast<const char*>(&intensity), sizeof(uint8_t));
    }

    //T_W_B
//    Eigen::Vector3d translation = poseMapp.pos.cast<double>();
//    Eigen::Quaterniond quaternion = poseMapp.rot.cast<double>();
    double tx = translation[0];
    double ty = translation[1];
    double tz = translation[2];
    double qx = quaternion.x();
    double qy = quaternion.y();
    double qz = quaternion.z();
    double qw = quaternion.w();
    ofs.write(reinterpret_cast<const char*>(&tx), sizeof(double));
    ofs.write(reinterpret_cast<const char*>(&ty), sizeof(double));
    ofs.write(reinterpret_cast<const char*>(&tz), sizeof(double));
    ofs.write(reinterpret_cast<const char*>(&qx), sizeof(double));
    ofs.write(reinterpret_cast<const char*>(&qy), sizeof(double));
    ofs.write(reinterpret_cast<const char*>(&qz), sizeof(double));
    ofs.write(reinterpret_cast<const char*>(&qw), sizeof(double));

    //GPSReadings
    int sizeOfGPS = gpsVec.size();
    ofs.write(reinterpret_cast<const char*>(&sizeOfGPS), sizeof(int));

    for (auto p : gpsVec) {

        double timestamp;
        Eigen::Vector3d lla;
        Eigen::Matrix3d covariance;
        GPSReading::COV_TYPE cov_type;
        GPSReading::FIX_TYPE fix_type;
        double heading;
        double headingCov;

        ofs.write(reinterpret_cast<const char*>(&p.timestamp), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&p.lla(0)), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&p.lla(1)), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&p.lla(2)), sizeof(double));

        ofs.write(reinterpret_cast<const char*>(&p.covariance(0,0)), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&p.covariance(0,1)), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&p.covariance(0,2)), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&p.covariance(1,0)), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&p.covariance(1,1)), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&p.covariance(1,2)), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&p.covariance(2,0)), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&p.covariance(2,1)), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&p.covariance(2,2)), sizeof(double));

        ofs.write(reinterpret_cast<const char*>(&p.cov_type), sizeof(char));
        ofs.write(reinterpret_cast<const char*>(&p.fix_type), sizeof(char));
        ofs.write(reinterpret_cast<const char*>(&p.heading), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&p.headingCov), sizeof(double));

    }
}

void save()
{
    while(1)
    {
        while (!key_scan_buf.empty())
        {
            mSave.lock();

            outputRMap(std::get<0>(key_scan_buf.front()), std::get<1>(key_scan_buf.front()), std::get<2>(key_scan_buf.front()), std::get<3>(key_scan_buf.front()), std::get<4>(key_scan_buf.front()),
                       std::get<5>(key_scan_buf.front()), std::get<6>(key_scan_buf.front()), std::get<7>(key_scan_buf.front()), std::get<8>(key_scan_buf.front()));
			
			// void outputRMap(const int &scanId, const int &keyScanId, const double &time,
            //     const pcl::PointCloud<pcl::PointXYZI> &fullRect, const pcl::PointCloud<pcl::PointXYZI> &lessSharpRect, const pcl::PointCloud<pcl::PointXYZI> &lessFlatRect,
            //     const Eigen::Vector3d& translation,const Eigen::Quaterniond& quaternion,std::vector<GPSReading>& gpsVec)
			if(1)
			{
				std::string str = strOut + "/keyscan-" + std::to_string(std::get<0>(key_scan_buf.front())) + ".txt";

				FILE *pFile = fopen(str.data(), "w");

				if (pFile) {
					for (int i = 0; i < std::get<3>(key_scan_buf.front()).size(); i++) {
					
						pcl::PointXYZI &pt = std::get<3>(key_scan_buf.front())[i];

						fprintf(pFile, "%f,%f,%f,%f\n", pt.x, pt.y, pt.z, pt.intensity);
						
					}

					fclose(pFile);
				}
			}
			// save submap
			if(0)
			{
				std::string str = strOut + "/submap-" + std::to_string(keyscanID) + ".txt";

				FILE *pFile = fopen(str.data(), "w");

				// output timestamp first
	//                fprintf(pFile, "%lf,%d\n", timeLaserOdometry, frameCount);

				if (pFile) {
					for (int i = 0; i < submap.size(); i++) {
						pcl::PointCloud<pcl::PointXYZI> pcl = submap[i];

						for (int j = 0; j < pcl.size(); j++) {
							pcl::PointXYZI &pt = pcl[j];

							fprintf(pFile, "%f,%f,%f,%f\n", pt.x, pt.y, pt.z, pt.intensity);
						}
					}

					fclose(pFile);
				}
			}
            
            key_scan_buf.pop();

            mSave.unlock();
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void process()
{
	while(1)
	{
		while (!cornerLastBuf.empty() && !surfLastBuf.empty() &&
			!fullResBuf.empty() && !odometryBuf.empty())
		{
			mBuf.lock();
			while (!cornerLastBuf.empty() && cornerLastBuf.front()->header.stamp.toSec() < odometryBuf.front()->header.stamp.toSec())
                cornerLastBuf.pop();
			if (cornerLastBuf.empty())
			{
				mBuf.unlock();
				break;
			}

			while (!surfLastBuf.empty() && surfLastBuf.front()->header.stamp.toSec() < odometryBuf.front()->header.stamp.toSec())
				surfLastBuf.pop();
			if (surfLastBuf.empty())
			{
				mBuf.unlock();
				break;
			}

			while (!fullResBuf.empty() && fullResBuf.front()->header.stamp.toSec() < odometryBuf.front()->header.stamp.toSec())
				fullResBuf.pop();
			if (fullResBuf.empty())
			{
				mBuf.unlock();
				break;
			}

			timeLaserCloudCornerLast = cornerLastBuf.front()->header.stamp.toSec();
			timeLaserCloudSurfLast = surfLastBuf.front()->header.stamp.toSec();
			timeLaserCloudFullRes = fullResBuf.front()->header.stamp.toSec();
			timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();

			if (timeLaserCloudCornerLast != timeLaserOdometry ||
				timeLaserCloudSurfLast != timeLaserOdometry ||
				timeLaserCloudFullRes != timeLaserOdometry)
			{
				printf("time corner %f surf %f full %f odom %f \n", timeLaserCloudCornerLast, timeLaserCloudSurfLast, timeLaserCloudFullRes, timeLaserOdometry);
				printf("laser mapping unsync messeage!");
				mBuf.unlock();
				break;
			}

			laserCloudCornerLast->clear();
			pcl::fromROSMsg(*cornerLastBuf.front(), *laserCloudCornerLast);
			cornerLastBuf.pop();

			laserCloudSurfLast->clear();
			pcl::fromROSMsg(*surfLastBuf.front(), *laserCloudSurfLast);
			surfLastBuf.pop();

			laserCloudFullRes->clear();
			pcl::fromROSMsg(*fullResBuf.front(), *laserCloudFullRes);
			fullResBuf.pop();

			q_wodom_curr.x() = odometryBuf.front()->pose.pose.orientation.x;
			q_wodom_curr.y() = odometryBuf.front()->pose.pose.orientation.y;
			q_wodom_curr.z() = odometryBuf.front()->pose.pose.orientation.z;
			q_wodom_curr.w() = odometryBuf.front()->pose.pose.orientation.w;
			t_wodom_curr.x() = odometryBuf.front()->pose.pose.position.x;
			t_wodom_curr.y() = odometryBuf.front()->pose.pose.position.y;
			t_wodom_curr.z() = odometryBuf.front()->pose.pose.position.z;
			odometryBuf.pop();

			while(!odometryBuf.empty())
			{
                odometryBuf.pop();
				printf("drop lidar frame in mapping for real time performance \n");
			}

			mBuf.unlock();

			std::vector<GPSReading> vecGPS;
			if(nGPS > 0)
            {
			    mBuf.lock();

                printf("gps buffer %d \n",gpsBuf.size());
                printf("gps time %lf %lf\n",gpsBuf.front().timestamp,timeLaserOdometry);
                while (!gpsBuf.empty() && gpsBuf.front().timestamp < timeLaserOdometry)
                {
                    vecGPS.push_back(gpsBuf.front());
                    printf("add gps time %lf \n",gpsBuf.front().timestamp);
                    gpsBuf.pop();
                }

			    mBuf.unlock();
            }

			TicToc t_whole;

			transformAssociateToMap();

			TicToc t_shift;
			int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;
			int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
			int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;

			if (t_w_curr.x() + 25.0 < 0)
				centerCubeI--;
			if (t_w_curr.y() + 25.0 < 0)
				centerCubeJ--;
			if (t_w_curr.z() + 25.0 < 0)
				centerCubeK--;

			while (centerCubeI < 3)
			{
				for (int j = 0; j < laserCloudHeight; j++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{ 
						int i = laserCloudWidth - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]; 
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; i >= 1; i--)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeI++;
				laserCloudCenWidth++;
			}

			while (centerCubeI >= laserCloudWidth - 3)
			{ 
				for (int j = 0; j < laserCloudHeight; j++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						int i = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; i < laserCloudWidth - 1; i++)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeI--;
				laserCloudCenWidth--;
			}

			while (centerCubeJ < 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						int j = laserCloudHeight - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; j >= 1; j--)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeJ++;
				laserCloudCenHeight++;
			}

			while (centerCubeJ >= laserCloudHeight - 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						int j = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; j < laserCloudHeight - 1; j++)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeJ--;
				laserCloudCenHeight--;
			}

			while (centerCubeK < 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int j = 0; j < laserCloudHeight; j++)
					{
						int k = laserCloudDepth - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; k >= 1; k--)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeK++;
				laserCloudCenDepth++;
			}

			while (centerCubeK >= laserCloudDepth - 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int j = 0; j < laserCloudHeight; j++)
					{
						int k = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; k < laserCloudDepth - 1; k++)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeK--;
				laserCloudCenDepth--;
			}

			int laserCloudValidNum = 0;
			int laserCloudSurroundNum = 0;

			for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
			{
				for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
				{
					for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
					{
						if (i >= 0 && i < laserCloudWidth &&
							j >= 0 && j < laserCloudHeight &&
							k >= 0 && k < laserCloudDepth)
						{ 
							laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
							laserCloudValidNum++;
							laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
							laserCloudSurroundNum++;
						}
					}
				}
			}

			laserCloudCornerFromMap->clear();
			laserCloudSurfFromMap->clear();
			for (int i = 0; i < laserCloudValidNum; i++)
			{
				*laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
				*laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
			}
			int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
			int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();


			pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
			downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
			downSizeFilterCorner.filter(*laserCloudCornerStack);
			int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

			pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
			downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
			downSizeFilterSurf.filter(*laserCloudSurfStack);
			int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

			// printf("map prepare time %f ms\n", t_shift.toc());
			// printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum);
			if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)
			{
				TicToc t_opt;
				TicToc t_tree;
				kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
				kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
				printf("build tree time %f ms \n", t_tree.toc());

				for (int iterCount = 0; iterCount < 5; iterCount++)
				{
					//ceres::LossFunction *loss_function = NULL;
					ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
					ceres::LocalParameterization *q_parameterization =
						new ceres::EigenQuaternionParameterization();
					ceres::Problem::Options problem_options;

					ceres::Problem problem(problem_options);
					problem.AddParameterBlock(parameters, 4, q_parameterization);
					problem.AddParameterBlock(parameters + 4, 3);

					TicToc t_data;
					int corner_num = 0;

					for (int i = 0; i < laserCloudCornerStackNum; i++)
					{
						pointOri = laserCloudCornerStack->points[i];
						//double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
						pointAssociateToMap(&pointOri, &pointSel);
						kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis); 

						if (pointSearchSqDis[4] < 1.0)
						{ 
							std::vector<Eigen::Vector3d> nearCorners;
							Eigen::Vector3d center(0, 0, 0);
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
													laserCloudCornerFromMap->points[pointSearchInd[j]].y,
													laserCloudCornerFromMap->points[pointSearchInd[j]].z);
								center = center + tmp;
								nearCorners.push_back(tmp);
							}
							center = center / 5.0;

							Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
							for (int j = 0; j < 5; j++)
							{
								Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
								covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
							}

							Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

							// if is indeed line feature
							// note Eigen library sort eigenvalues in increasing order
							Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
							if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
							{ 
								Eigen::Vector3d point_on_line = center;
								Eigen::Vector3d point_a, point_b;
								point_a = 0.1 * unit_direction + point_on_line;
								point_b = -0.1 * unit_direction + point_on_line;

								ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
								problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
								corner_num++;	
							}							
						}
						/*
						else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
						{
							Eigen::Vector3d center(0, 0, 0);
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
													laserCloudCornerFromMap->points[pointSearchInd[j]].y,
													laserCloudCornerFromMap->points[pointSearchInd[j]].z);
								center = center + tmp;
							}
							center = center / 5.0;	
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
							ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
							problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
						}
						*/
					}

					int surf_num = 0;
					for (int i = 0; i < laserCloudSurfStackNum; i++)
					{
						pointOri = laserCloudSurfStack->points[i];
						//double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
						pointAssociateToMap(&pointOri, &pointSel);
						kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

						Eigen::Matrix<double, 5, 3> matA0;
						Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
						if (pointSearchSqDis[4] < 1.0)
						{
							
							for (int j = 0; j < 5; j++)
							{
								matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
								matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
								matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
								//printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
							}
							// find the norm of plane
							Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
							double negative_OA_dot_norm = 1 / norm.norm();
							norm.normalize();

							// Here n(pa, pb, pc) is unit norm of plane
							bool planeValid = true;
							for (int j = 0; j < 5; j++)
							{
								// if OX * n > 0.2, then plane is not fit well
								if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
										 norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
										 norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
								{
									planeValid = false;
									break;
								}
							}
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
							if (planeValid)
							{
								ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
								problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
								surf_num++;
							}
						}
						/*
						else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
						{
							Eigen::Vector3d center(0, 0, 0);
							for (int j = 0; j < 5; j++)
							{
								Eigen::Vector3d tmp(laserCloudSurfFromMap->points[pointSearchInd[j]].x,
													laserCloudSurfFromMap->points[pointSearchInd[j]].y,
													laserCloudSurfFromMap->points[pointSearchInd[j]].z);
								center = center + tmp;
							}
							center = center / 5.0;	
							Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
							ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
							problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
						}
						*/
					}

					//printf("corner num %d used corner num %d \n", laserCloudCornerStackNum, corner_num);
					//printf("surf num %d used surf num %d \n", laserCloudSurfStackNum, surf_num);

					// printf("mapping data assosiation time %f ms \n", t_data.toc());

					TicToc t_solver;
					ceres::Solver::Options options;
					options.linear_solver_type = ceres::DENSE_QR;
					options.max_num_iterations = 4;
					options.minimizer_progress_to_stdout = false;
					options.check_gradients = false;
					options.gradient_check_relative_precision = 1e-4;
					ceres::Solver::Summary summary;
					ceres::Solve(options, &problem, &summary);
					// printf("mapping solver time %f ms \n", t_solver.toc());

					//printf("time %f \n", timeLaserOdometry);
					//printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
					//printf("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1], parameters[2],
					//	   parameters[4], parameters[5], parameters[6]);
				}
				// printf("mapping optimization time %f \n", t_opt.toc());
			}
			else
			{
				ROS_WARN("time Map corner and surf num are not enough");
			}
			transformUpdate();

			TicToc t_add;
			for (int i = 0; i < laserCloudCornerStackNum; i++)
			{
				pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

				int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
				int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
				int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

				if (pointSel.x + 25.0 < 0)
					cubeI--;
				if (pointSel.y + 25.0 < 0)
					cubeJ--;
				if (pointSel.z + 25.0 < 0)
					cubeK--;

				if (cubeI >= 0 && cubeI < laserCloudWidth &&
					cubeJ >= 0 && cubeJ < laserCloudHeight &&
					cubeK >= 0 && cubeK < laserCloudDepth)
				{
					int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
					laserCloudCornerArray[cubeInd]->push_back(pointSel);
				}
			}

			for (int i = 0; i < laserCloudSurfStackNum; i++)
			{
				pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

				int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
				int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
				int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

				if (pointSel.x + 25.0 < 0)
					cubeI--;
				if (pointSel.y + 25.0 < 0)
					cubeJ--;
				if (pointSel.z + 25.0 < 0)
					cubeK--;

				if (cubeI >= 0 && cubeI < laserCloudWidth &&
					cubeJ >= 0 && cubeJ < laserCloudHeight &&
					cubeK >= 0 && cubeK < laserCloudDepth)
				{
					int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
					laserCloudSurfArray[cubeInd]->push_back(pointSel);
				}
			}
			// printf("add points time %f ms\n", t_add.toc());

			
			TicToc t_filter;
			for (int i = 0; i < laserCloudValidNum; i++)
			{
				int ind = laserCloudValidInd[i];

				pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
				downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
				downSizeFilterCorner.filter(*tmpCorner);
				laserCloudCornerArray[ind] = tmpCorner;

				pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
				downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
				downSizeFilterSurf.filter(*tmpSurf);
				laserCloudSurfArray[ind] = tmpSurf;
			}
			// printf("filter time %f ms \n", t_filter.toc());
			
			TicToc t_pub;
			//publish surround map for every 5 frame
			if (frameCount % 5 == 0)
			{
				laserCloudSurround->clear();
				for (int i = 0; i < laserCloudSurroundNum; i++)
				{
					int ind = laserCloudSurroundInd[i];
					*laserCloudSurround += *laserCloudCornerArray[ind];
					*laserCloudSurround += *laserCloudSurfArray[ind];
				}

				sensor_msgs::PointCloud2 laserCloudSurround3;
				pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
				laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				laserCloudSurround3.header.frame_id = "/camera_init";
				pubLaserCloudSurround.publish(laserCloudSurround3);
			}

			if (frameCount % 20 == 0)
			{
				pcl::PointCloud<PointType> laserCloudMap;
				for (int i = 0; i < 4851; i++)
				{
					laserCloudMap += *laserCloudCornerArray[i];
					laserCloudMap += *laserCloudSurfArray[i];
				}
				sensor_msgs::PointCloud2 laserCloudMsg;
				pcl::toROSMsg(laserCloudMap, laserCloudMsg);
				laserCloudMsg.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				laserCloudMsg.header.frame_id = "/camera_init";
				pubLaserCloudMap.publish(laserCloudMsg);
			}

			int laserCloudFullResNum = laserCloudFullRes->points.size();
			for (int i = 0; i < laserCloudFullResNum; i++)
			{
				pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
			}

			sensor_msgs::PointCloud2 laserCloudFullRes3;
			pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
			laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
			laserCloudFullRes3.header.frame_id = "/camera_init";
			pubLaserCloudFullRes.publish(laserCloudFullRes3);

			// printf("mapping pub time %f ms \n", t_pub.toc());

			printf("whole mapping time %f ms +++++\n", t_whole.toc());

			nav_msgs::Odometry odomAftMapped;
			odomAftMapped.header.frame_id = "/camera_init";
			odomAftMapped.child_frame_id = "/aft_mapped";
			odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
			odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
			odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
			odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
			odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
			odomAftMapped.pose.pose.position.x = t_w_curr.x();
			odomAftMapped.pose.pose.position.y = t_w_curr.y();
			odomAftMapped.pose.pose.position.z = t_w_curr.z();
			pubOdomAftMapped.publish(odomAftMapped);
			
			Eigen::Vector3d diff_p = t_w_curr - t_w_pre;

            // if (rmapOut && frameCount > 0 && (frameCount % savefreq == 0))
            if (rmapOut && frameCount > 0 && (diff_p.norm() > keyframe_dis))
            {
                mSave.lock();
                key_scan_buf.push(std::make_tuple(frameCount,keyscanID,timeLaserOdometry,*laserCloudFullRes,*laserCloudCornerLast,*laserCloudSurfLast,t_w_curr,q_w_curr,vecGPS));
                mSave.unlock();

                keyscanID++;

				t_w_pre = t_w_curr;
            } else
            {
                pcl::PointCloud<pcl::PointXYZI> pcd;
                pcl::copyPointCloud(*laserCloudFullRes,pcd);
                submap.push(pcd);
            }

			geometry_msgs::PoseStamped laserAfterMappedPose;
			laserAfterMappedPose.header = odomAftMapped.header;
			laserAfterMappedPose.pose = odomAftMapped.pose.pose;
			laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
			laserAfterMappedPath.header.frame_id = "/camera_init";
			laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
			pubLaserAfterMappedPath.publish(laserAfterMappedPath);

			static tf::TransformBroadcaster br;
			tf::Transform transform;
			tf::Quaternion q;
			transform.setOrigin(tf::Vector3(t_w_curr(0),
											t_w_curr(1),
											t_w_curr(2)));
			q.setW(q_w_curr.w());
			q.setX(q_w_curr.x());
			q.setY(q_w_curr.y());
			q.setZ(q_w_curr.z());
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "/camera_init", "/aft_mapped"));

			frameCount++;
		}
		std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
	}
}


void save_map()
{
	mSave.lock();

	std::string str = strOut + "/sparse_map.txt";

	FILE* pFile = fopen(str.data(),"w");

	pcl::PointCloud<PointType> laserCloudMap;
	for (int i = 0; i < 4851; i++)
	{
		laserCloudMap += *laserCloudCornerArray[i];
		laserCloudMap += *laserCloudSurfArray[i];
	}

	for(int i = 0; i< laserCloudMap.size();i++)
	{
		PointType& pt = laserCloudMap[i];

		fprintf(pFile,"%f,%f,%f\n",pt.x,pt.y,pt.z);
	}
	fclose(pFile);

	mSave.unlock();

}

void save_callback(const std_msgs::BoolConstPtr &restart_msg)
{
	ROS_WARN("save map!");
	save_map();

	ROS_INFO("save map done!");
}

void gpsfixCallback(const sensor_msgs::NavSatFix::ConstPtr &gpsfix)
{
    if(nGPS != 1)
        return;
    GPSReading gps;

    gps.timestamp = gpsfix->header.stamp.toSec();
    gps.lla[0] = gpsfix->latitude;
    gps.lla[1] = gpsfix->longitude;
    gps.lla[2] = gpsfix->altitude;
    gps.covariance(0, 0) = gpsfix->position_covariance[0];
    gps.covariance(0, 1) = gpsfix->position_covariance[1];
    gps.covariance(0, 2) = gpsfix->position_covariance[2];
    gps.covariance(1, 0) = gpsfix->position_covariance[3];
    gps.covariance(1, 1) = gpsfix->position_covariance[4];
    gps.covariance(1, 2) = gpsfix->position_covariance[5];
    gps.covariance(2, 0) = gpsfix->position_covariance[6];
    gps.covariance(2, 1) = gpsfix->position_covariance[7];
    gps.covariance(2, 2) = gpsfix->position_covariance[8];
    gps.fix_type = (GPSReading::FIX_TYPE) gpsfix->status.status;
    gps.cov_type = (GPSReading::COV_TYPE) gpsfix->position_covariance_type;

    gpsBuf.push(gps);
}

void gpsodomCallback(const nav_msgs::Odometry::ConstPtr &gps_odo)
{
    if(nGPS != 2)
        return;

    GPSReading gps;

    gps.timestamp = gps_odo->header.stamp.toSec();

    gps.lla[0] = gps_odo->pose.pose.position.x;
    gps.lla[1] = gps_odo->pose.pose.position.y;
    gps.lla[2] = gps_odo->pose.pose.position.z;
    gps.covariance(0, 0) = gps_odo->pose.covariance[1]*gps_odo->pose.covariance[1];
    gps.covariance(0, 1) = 0;
    gps.covariance(0, 2) = 0;
    gps.covariance(1, 0) = 0;
    gps.covariance(1, 1) = gps_odo->pose.covariance[2]*gps_odo->pose.covariance[2];
    gps.covariance(1, 2) = 0;
    gps.covariance(2, 0) = 0;
    gps.covariance(2, 1) = 0;
    gps.covariance(2, 2) = gps_odo->pose.covariance[3]*gps_odo->pose.covariance[3];
    gps.fix_type = (GPSReading::FIX_TYPE::STATUS_FIX);
    gps.cov_type = (GPSReading::COV_TYPE::COVARIANCE_TYPE_DIAGONAL_KNOWN);

    gpsBuf.push(gps);
}

void command()
{
	while(1)
	{
		char c = getchar();
		if (c == 's')
		{
			ROS_WARN("save map!");
			save_map();

			ROS_INFO("save map done!");
		}
		std::chrono::milliseconds dura(5);
		std::this_thread::sleep_for(dura);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laserMapping");
	ros::NodeHandle nh;

	float lineRes = 0;
	float planeRes = 0;
	nh.param<float>("mapping_line_resolution", lineRes, 0.4);
	nh.param<float>("mapping_plane_resolution", planeRes, 0.8);
	printf("line resolution %f plane resolution %f \n", lineRes, planeRes);
	downSizeFilterCorner.setLeafSize(lineRes, lineRes,lineRes);
	downSizeFilterSurf.setLeafSize(planeRes, planeRes, planeRes);

	nh.getParam("out_path", strOut);
	nh.param<int>("save_freq",savefreq,5);
	nh.param<int>("rmap",rmapOut,0);
    nh.param<int>("gpsout", nGPS, 0);

    printf("rmap %d \n", rmapOut);
    printf("gpsout %d \n", nGPS);

    nh.param<int>("submap_binsize",nSubmapBinSize,20);
    nh.param<double>("keyframe_dis",keyframe_dis,1);

	if(rmapOut)
    {
        ofs.open(strOut + "/global.rmap", std::ios::binary);
        double PBG0, PBG1, PBG2;
        double RBG00, RBG01, RBG02, RBG10, RBG11, RBG12, RBG20, RBG21, RBG22;
        PBG0 = PBG1 = PBG2 = 0.0;
        RBG00 = RBG01 = RBG02 = RBG10 = RBG11 = RBG12 = RBG20 = RBG21 = RBG22 = 0.0;

        if(nGPS > 0)
        {
            double tmpx,tmpy,tmpz;

            nh.param<double>("imu_x",tmpx,0);
            nh.param<double>("imu_y",tmpy,0);
            nh.param<double>("imu_z",tmpz,0);

            Eigen::Vector3d PBI(tmpx,tmpy,tmpz);

            nh.param<double>("imu_rx",tmpx,0);
            nh.param<double>("imu_ry",tmpy,0);
            nh.param<double>("imu_rz",tmpz,0);

            Eigen::Quaterniond QBI = Eigen::Quaterniond(Eigen::AngleAxisd(tmpz * M_PI / 180.0f, Eigen::Vector3d::UnitZ())
                                                        * Eigen::AngleAxisd(tmpy * M_PI / 180.0f, Eigen::Vector3d::UnitY())
                                                        * Eigen::AngleAxisd(tmpx * M_PI / 180.0f, Eigen::Vector3d::UnitX()));

            Eigen::Matrix3d RBI = QBI.toRotationMatrix();

            nh.param<double>("gps_x",tmpx,0);
            nh.param<double>("gps_y",tmpy,0);
            nh.param<double>("gps_z",tmpz,0);

            Eigen::Vector3d PIG(tmpx,tmpy,tmpz);

            nh.param<double>("gps_rx",tmpx,0);
            nh.param<double>("gps_ry",tmpy,0);
            nh.param<double>("gps_rz",tmpz,0);

            Eigen::Quaterniond QIG = Eigen::Quaterniond(Eigen::AngleAxisd(tmpz * M_PI / 180.0f, Eigen::Vector3d::UnitZ())
                                                        * Eigen::AngleAxisd(tmpy * M_PI / 180.0f, Eigen::Vector3d::UnitY())
                                                        * Eigen::AngleAxisd(tmpx * M_PI / 180.0f, Eigen::Vector3d::UnitX()));

            Eigen::Matrix3d RIG = QIG.toRotationMatrix();

            // from gps to body
            Eigen::Vector3d PBG = RBI * PIG + PBI;
            Eigen::Matrix3d RBG = RBI * RIG;

            PBG0 = PBG(0);
            PBG1 = PBG(1);
            PBG2 = PBG(2);

            RBG00 = RBG(0,0);
            RBG01 = RBG(0,1);
            RBG02 = RBG(0,2);
            RBG10 = RBG(1,0);
            RBG11 = RBG(1,1);
            RBG12 = RBG(1,2);
            RBG20 = RBG(2,0);
            RBG21 = RBG(2,1);
            RBG22 = RBG(2,2);
        }

        printf("PBG %f %f %f \n", PBG0, PBG1, PBG2);
        printf("RBG:%lf,%lf,%lf\n", RBG00, RBG01, RBG02);
        printf("RBG:%lf,%lf,%lf\n", RBG10, RBG11, RBG12);
        printf("RBG:%lf,%lf,%lf\n", RBG20, RBG21, RBG22);

        ofs.write(reinterpret_cast<const char*>(&PBG0), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&PBG1), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&PBG2), sizeof(double));

        ofs.write(reinterpret_cast<const char*>(&RBG00), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&RBG01), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&RBG02), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&RBG10), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&RBG11), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&RBG12), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&RBG20), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&RBG21), sizeof(double));
        ofs.write(reinterpret_cast<const char*>(&RBG22), sizeof(double));
    }

	ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100, laserCloudCornerLastHandler);

	ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100, laserCloudSurfLastHandler);

	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, laserOdometryHandler);

	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100, laserCloudFullResHandler);

	ros::Subscriber sub_save = nh.subscribe("/save", 5, save_callback);

    ros::Subscriber sub3 = nh.subscribe("/GPS_fix", 5, gpsfixCallback);
    ros::Subscriber sub6 = nh.subscribe("/GPS_odom",5, gpsodomCallback);

	pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);

	pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_map", 100);

	pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);

	pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);

	pubOdomAftMappedHighFrec = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init_high_frec", 100);

	pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);

	for (int i = 0; i < laserCloudNum; i++)
	{
		laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
		laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
	}

	std::thread mapping_process{process};
	std::thread saving_process{save};

	std::thread keyboard_command_process;
	keyboard_command_process = std::thread(command);

	ros::spin();

	return 0;
}