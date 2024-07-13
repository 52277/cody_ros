# include "chassis_driver/cody_driver.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "chassis_driver");
    cody_driver::CODY CODY03;
    CODY03.run();

    return 0;
}
