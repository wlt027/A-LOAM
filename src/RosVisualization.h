//
// Created by wlt-zh on 9/14/18.
//

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>

#include <pangolin/pangolin.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <thread>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

using namespace std;

namespace vis {

    struct traj_point {

        Eigen::Vector3d P;
        Eigen::Quaterniond Q;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

    class RosVisualization {
    public:
        explicit RosVisualization();

        /** \brief Setup component in active mode.
         *
         * @param node the ROS node handle
         * @param privateNode the private ROS node handle
         */
        bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

        /** \brief Process incoming messages in a loop until shutdown (used in active mode). */
        void spin();

        /** \brief Handler method for a new full resolution cloud.
         *
         * @param laserCloudFullResMsg the new full resolution cloud message
         */
        void laserCloudSurroundHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudSurroundDSMsg);

        /** \brief Handler method for a new full resolution cloud.
         *
         * @param laserCloudFullResMsg the new full resolution cloud message
         */
        void laserCloudMapHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMapMsg);

        /** \brief Handler method for a new full resolution cloud.
         *
         * @param laserCloudFullResMsg the new full resolution cloud message
         */
        void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg);

        /** \brief Handler method for laser odometry messages.
         *
         * @param laserOdometry the new laser odometry
         */
        void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);

        /** \brief Handler method for mapping odometry messages.
         *
         * @param odomAftMapped the new mapping odometry
         */
        void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped);

    public:
        void run();

        bool isRequiredStop();

        void setStop();

        bool waitForFinish();

    protected:
        // publish restart message
        void pubRestartMessage();

        // publish save message
        void pubSaveMessage();

        void clearState();

    private:
        void drawSubMapPoints();

        void drawAxis(pangolin::OpenGlMatrix &Twi);

        void getCurrentAxisPose(pangolin::OpenGlMatrix &M);


        void drawTrajectory();

        void drawCurrentScan();

    private:

        // loam
        ros::Subscriber _subLaserOdometry;    ///< (high frequency) laser odometry subscriber
        ros::Subscriber _subOdomAftMapped;    ///< (low frequency) mapping odometry subscriber

        ros::Subscriber _subLaserCloudSurround;    ///< map cloud message subscriber
        ros::Subscriber _subLaserCloudMap;    ///< map cloud message subscriber
        ros::Subscriber _subLaserCloudFullRes;     ///< current full resolution cloud message subscriber

        // restart
        ros::Publisher _pubRestart;

        // save
        ros::Publisher _pubSave;

        // optimize
        ros::Publisher _pubOptimize;

    private:

        std::shared_ptr<std::thread> pangolin_thread_;

        bool required_stop_;
        bool is_finished_;


        std::mutex mutex_pose_;
        std::mutex mutex_cur_pose_;
        std::mutex mutex_laser_cloud_;
        std::mutex mutex_laser_cloud_surround_;

        std::mutex mutex_stop_;

        bool m_bShowIntensity;
        bool m_bShowBigPointSize;

        //Scan
        pcl::PointCloud<pcl::PointXYZ>::Ptr _laserCloudFullRes;      ///< last full resolution cloud

        //Map
        pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurroundDS;     ///< down sampled

        traj_point curPQ;
        double curT;

        // body trajectory
        std::vector<std::pair<double,traj_point>> mTraj;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}