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
#include <aloam_velodyne/color.h>

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
        void initRotHandler(const nav_msgs::Odometry::ConstPtr& Odo);

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

        /** \brief Handler method for a tracked image.
         *
         * @param img_msg the new tracked image message
         */
        void trackedImageHandler(const sensor_msgs::Image::ConstPtr &img_msg);
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

        void drawCurrentImage(pangolin::GlTexture &gl_texture, cv::Mat &image);

        void drawTrajectory();

        void drawCurrentScan();

    private:

        // loam
        ros::Subscriber _subLaserOdometry;    ///< (high frequency) laser odometry subscriber
        ros::Subscriber _subOdomAftMapped;    ///< (low frequency) mapping odometry subscriber

        ros::Subscriber _subLaserCloudSurround;    ///< map cloud message subscriber
        ros::Subscriber _subLaserCloudMap;    ///< map cloud message subscriber
        ros::Subscriber _subLaserCloudFullRes;     ///< current full resolution cloud message subscriber

        ros::Subscriber _sub_track_img;
        ros::Subscriber _sub_init_rot;

        // restart
        ros::Publisher _pubRestart;

        // save
        ros::Publisher _pubSave;

    private:

        std::shared_ptr<std::thread> pangolin_thread_;

        bool required_stop_;
        bool is_finished_;

        // current image
        cv::Mat image_;
        cv::Size image_size_;

        std::mutex mutex_image_;
        std::mutex mutex_pose_;
        std::mutex mutex_cur_pose_;
        std::mutex mutex_laser_cloud_;
        std::mutex mutex_laser_cloud_surround_;

        std::mutex mutex_stop_;

        bool m_bShowIntensity;
        bool m_bShowBigPointSize;
        bool m_bOutput;

        //Scan
        pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudFullRes;      ///< last full resolution cloud

        //Map
        pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurroundDS;     ///< down sampled

        traj_point curPQ;

        double curT;

        int nScanCount;

        int nSaveFreq;
        // body trajectory
        std::vector<std::pair<double,traj_point>> mTraj;

        Eigen::Quaterniond initQ;

        std::string strOut;

        ColorRamp ramp;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}