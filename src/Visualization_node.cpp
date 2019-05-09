//
// Created by wlt-zh on 9/14/18.
//

//#include <ros/ros.h>
#include "RosVisualization.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualizer");
    ros::NodeHandle n("~");

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::NodeHandle node;

    vis::RosVisualization globalVis;

    if (globalVis.setup(node, n)) {
        // initialization successful
        globalVis.spin();
        globalVis.waitForFinish();
    }

    return 0;
}
