#ifndef CODY_DRIVER
#define CODY_DRIVER
#include <iostream>
#include <fstream>
#include <time.h>
#include <ros/ros.h>
#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"

#include "can_interfaces/CtrlCmd.h"
#include "can_interfaces/OdomFb.h"
#include "can_interfaces/MotorMotionFb.h"
#include "nav_msgs/Odometry.h"
#include <chrono>
#include <vector>
#include <deque>
#include <string>
using namespace itas109;
using namespace std::chrono_literals;
namespace cody_driver
{
    class MyListener : public CSerialPortListener
    {
    public:
        MyListener(CSerialPort *sp)
            : p_sp(sp){};

        void onReadEvent(const char *portName, unsigned int readBufferLen)
        {
            // LOG(INFO) << "bufferlen is " << readBufferLen;
            if (readBufferLen > 0)
            {
                char *data = new char[readBufferLen + 1]; // '\0'

                if (data)
                {
                    // read
                    int recLen = p_sp->readData(data, readBufferLen);

                    if (recLen > 0)
                    {
                        data[recLen] = '\0';
                        // printf("%s , Length: %d, Str: %s, Hex: %s\n", portName, recLen, data, char2hexstr(data, recLen).c_str());
                        s_buffer = char2hexstr(data, recLen);
                    }

                    delete[] data;
                    data = NULL;
                }
            }
        };

        std::string get_buffer(){return s_buffer;}

    private:
        CSerialPort *p_sp;
        std::string s_buffer;

        std::string char2hexstr(const char *str, int len)
        {
            static const char hexTable[17] = "0123456789ABCDEF";

            std::string result;
            for (int i = 0; i < len; ++i)
            {
                // result += "0x";
                result += hexTable[(unsigned char)str[i] / 16];
                result += hexTable[(unsigned char)str[i] % 16];
                // result += " ";
            }
            return result;
        }
    };
    class CODY
    {
        public:
        CODY();
        ~CODY();
        void run();

        private:
        //句柄
        ros::NodeHandle nh;
        //变量
        CSerialPort sp;
        int length;//单个信息长度
        int LOOP_RATE;
        int data_packet_start;//单个信息长度
        //发布者
        ros::Subscriber vel_sub;
        ros::Publisher cmd_feed_pub;
        ros::Publisher odom_feed_pub;
        ros::Publisher motor_feed_pub;
        ros::Publisher pose_feed_pub;
        //监听串口
        MyListener listener;
        //监听取出的字符串
        std::string buffer_s;
        //维护一个从控制节点下发的command，每次订阅更新这个数值。
        can_interfaces::CtrlCmd my_cmd;
        //发布的数据
        can_interfaces::CtrlCmd cmd_feedback;
        can_interfaces::OdomFb odom_feedback;
        can_interfaces::MotorMotionFb my_motor_motion;
        //维护的里程计
        nav_msgs::Odometry my_pose;
        double theta_now;
        int initial_odom;
        bool initial_odom_flag;
        //函数
        void initForROS();//初始化ROS
        void init_open_serial();//初始化并打开串口
        void read_serial();//对串口数据进行处理
        void spst_control();
        void publish_topic();
        void sub_command(const can_interfaces::CtrlCmd::ConstPtr& msg);
        void mode_enable();
        void calculate_pose();
    };

}

#endif