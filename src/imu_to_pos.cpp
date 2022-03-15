#include "imu_header.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh("~");
    ImuTransfer ImuTransfer(nh);
    while (ros::ok())
    {
        ros::spin();
    }

    return 0;
}
