#include "chassis_driver/cody_driver.h"
#include "chassis_driver/utils.h"

namespace cody_driver
{
    CODY::CODY()
    :nh("~")
    ,length(17)
    ,LOOP_RATE(100)
    ,data_packet_start(0)
    ,buffer_s("")
    ,listener(&sp)
    ,initial_odom_flag(false)
    {
        initForROS();
        init_open_serial();
        mode_enable();
    }
    CODY::~CODY()
    {
      std::string hexData = "AA 00 00 08 00 00 04 21 00 00 00 00 00 00 00 00";
      auto bytes0 = hexStringToBytes(hexData);
    
      sp.writeData(bytes0, 16);
      
      std::cerr << "mode unable" << std::endl;
    }

    void CODY::mode_enable()
    {
      std::string hexData = "AA 00 00 08 00 00 04 21 01 00 00 00 00 00 00 00";
      auto bytes0 = hexStringToBytes(hexData);
    
      sp.writeData(bytes0, 16);
      
      std::cerr << "mode enable" << std::endl;
    }
    void CODY::run()
    {
        ros::Rate loop_rate(LOOP_RATE);
        while(ros::ok())
        {
            read_serial();
            spst_control();
            calculate_pose();
            publish_topic();
            ros::spinOnce();
            loop_rate.sleep();
        }
        sp.close();
    }
    void CODY::initForROS()
    {
        vel_sub = nh.subscribe<can_interfaces::CtrlCmd>("ctrl_cmd", 10, &CODY::sub_command,this);
        cmd_feed_pub = nh.advertise<can_interfaces::CtrlCmd>("cmd_feed", 1);
        odom_feed_pub = nh.advertise<can_interfaces::OdomFb>("odom_feed", 1);
        motor_feed_pub = nh.advertise<can_interfaces::MotorMotionFb>("motor_feed", 1);
        pose_feed_pub = nh.advertise<nav_msgs::Odometry>("pose_feed", 1);
    }
    void CODY::init_open_serial()
    {
        bool find_it = false;
        //设置要打开的串口名称
        while(true)
        {
            std::vector<SerialPortInfo> m_availablePortsList = CSerialPortInfo::availablePortInfos();
            int availablePortCount = (int)m_availablePortsList.size();
            std::string port_name;
            for (int i = 1; i <= availablePortCount; ++i)
            {
                SerialPortInfo serialPortInfo = m_availablePortsList[i - 1];
                std::string spname = serialPortInfo.portName;
                std::string spdescription = serialPortInfo.description;
                // spname == "/dev/ttyUSB1" && 
                if(spdescription.substr(0,7) == "QinHeng")
                {
                    port_name = serialPortInfo.portName;
                    find_it = true;
                }
        
            }
            if(find_it)
            {
            sp.init(port_name.c_str(),
            itas109::BaudRate115200,
            itas109::ParityNone,
            itas109::DataBits8,
            itas109::StopOne,
            itas109::FlowNone,
            4096
            );
            std::cout << "try to open " << port_name<< std::endl;
            
            //串口设置timeout

            sp.setReadIntervalTimeout(0);
            //打开串口
            sp.open();
            printf("Open %s %s\n", port_name, sp.isOpen() ? "Success" : "Failed");
            printf("Code: %d, Message: %s\n", sp.getLastError(), sp.getLastErrorMsg());
            

            //检查串口const
            if(sp.isOpen())
            {
                std::cout << "sp open\n";
                return;
            }

            }

            else
            {
            std::cout <<"No canusb now\n";
            continue;
            }
        }
    }

    void CODY::calculate_pose()
    {
      if(initial_odom_flag == false)
      {
        initial_odom = odom_feedback.odom_all;
        my_pose.pose.pose.position.x = 0;
        my_pose.pose.pose.position.y = 0;
        initial_odom_flag = true;
      }
      double wheelbase = 0.68;//轴距为0.68m
      double v_now = cmd_feedback.ctrl_cmd_velocity *0.00066;
      double steering_theta = cmd_feedback.ctrl_cmd_steering * 0.06 * M_PI / 180;
      std::cerr << "steering_theta is " << steering_theta << std::endl;
      my_pose.twist.twist.angular.x = v_now / wheelbase * tan(steering_theta);
      std::cerr << "twist_angular is " << my_pose.twist.twist.angular.x << std::endl;
      my_pose.twist.twist.angular.y = 0;
      my_pose.twist.twist.angular.z = 0;
      theta_now = theta_now + my_pose.twist.twist.angular.x * 0.01;
      std::cerr << "theta_now is " << theta_now << std::endl;
      std::cerr << "v_now is " << v_now << std::endl;
      my_pose.twist.twist.linear.x = v_now * cos(theta_now);
      my_pose.twist.twist.linear.y = v_now * sin(theta_now);
      my_pose.pose.pose.orientation.w = cos(theta_now/2);
      my_pose.pose.pose.orientation.x = 0;
      my_pose.pose.pose.orientation.y = 0;
      my_pose.pose.pose.orientation.z = sin(theta_now/2);
      int odom_now = odom_feedback.odom_all - initial_odom;
      my_pose.pose.pose.position.x += my_pose.twist.twist.linear.x * 0.01;
      my_pose.pose.pose.position.y += my_pose.twist.twist.linear.y * 0.01;
      std::cerr << "positiony is " << my_pose.pose.pose.position.y << std::endl;
      my_pose.pose.pose.position.z = 0;
      
      
    }


    void CODY::read_serial()
    {
        sp.connectReadEvent(&listener);
        buffer_s = listener.get_buffer();
        // LOG(INFO) << "buffer_s is:" << buffer_s;
        std::vector<size_t> buffer_index = {};
        bool start_flag = find_all(buffer_index, buffer_s, "AA");

        // if(buffer_s.find("0241") < 1000)
        // LOG(INFO) << "0241 is" << buffer_s.find("0241");


        if (start_flag)
        {
          data_packet_start = buffer_index.back();
          // LOG(INFO) << "data_start is " << data_packet_start;
          
          //里程计反馈
          if(buffer_s.find("0311")-data_packet_start == 12)
          {
            // 16代表第十六位，每一位代表十六进制表示，参考包模式定义
             odom_feedback.odom_all = (((uint8_t)std::stoi(buffer_s.substr(16 + data_packet_start, 2), nullptr, 16)) << 24 )| (((uint8_t)std::stoi(buffer_s.substr(18 + data_packet_start, 2), nullptr, 16)) << 16 ) | (((uint8_t)std::stoi(buffer_s.substr(20 + data_packet_start, 2), nullptr, 16)) << 8 ) | ((uint8_t)std::stoi(buffer_s.substr(22 + data_packet_start, 2), nullptr, 16));
          }


          //速度转速反馈,右前左前右后左后分别是250,251,252,253
          //速度转速反馈,右前左前右后左后分别是250,251,252,253
          else if(buffer_s.find("0250")-data_packet_start == 12)
          {
            my_motor_motion.motor_rpm_rf = (((uint8_t)std::stoi(buffer_s.substr(16 + data_packet_start, 2), nullptr, 16)) << 8 )| (uint8_t)std::stoi(buffer_s.substr(18 + data_packet_start, 2), nullptr, 16);
            my_motor_motion.motor_location_rf = (((uint8_t)std::stoi(buffer_s.substr(24 + data_packet_start, 2), nullptr, 16)) << 24 )| (((uint8_t)std::stoi(buffer_s.substr(26 + data_packet_start, 2), nullptr, 16)) << 16 ) | (((uint8_t)std::stoi(buffer_s.substr(28 + data_packet_start, 2), nullptr, 16)) << 8 ) | ((uint8_t)std::stoi(buffer_s.substr(30 + data_packet_start, 2), nullptr, 16));
          }
          else if(buffer_s.find("0251")-data_packet_start == 12)
          {
            // LOG(INFO) << "find 251";
            my_motor_motion.motor_rpm_lf = (((uint8_t)std::stoi(buffer_s.substr(16 + data_packet_start, 2), nullptr, 16)) << 8 )| (uint8_t)std::stoi(buffer_s.substr(18 + data_packet_start, 2), nullptr, 16);
            my_motor_motion.motor_location_lf = (((uint8_t)std::stoi(buffer_s.substr(24 + data_packet_start, 2), nullptr, 16)) << 24 )| (((uint8_t)std::stoi(buffer_s.substr(26 + data_packet_start, 2), nullptr, 16)) << 16 ) | (((uint8_t)std::stoi(buffer_s.substr(28 + data_packet_start, 2), nullptr, 16)) << 8 ) | ((uint8_t)std::stoi(buffer_s.substr(30 + data_packet_start, 2), nullptr, 16));
          }
          else if(buffer_s.find("0252")-data_packet_start == 12)
          {
            my_motor_motion.motor_rpm_rb = (((uint8_t)std::stoi(buffer_s.substr(16 + data_packet_start, 2), nullptr, 16)) << 8 )| (uint8_t)std::stoi(buffer_s.substr(18 + data_packet_start, 2), nullptr, 16);
            my_motor_motion.motor_location_rb = (((uint8_t)std::stoi(buffer_s.substr(24 + data_packet_start, 2), nullptr, 16)) << 24 )| (((uint8_t)std::stoi(buffer_s.substr(26 + data_packet_start, 2), nullptr, 16)) << 16 ) | (((uint8_t)std::stoi(buffer_s.substr(28 + data_packet_start, 2), nullptr, 16)) << 8 ) | ((uint8_t)std::stoi(buffer_s.substr(30 + data_packet_start, 2), nullptr, 16));
          }
          else if(buffer_s.find("0253")-data_packet_start == 12)
          {
            my_motor_motion.motor_rpm_lb = (((uint8_t)std::stoi(buffer_s.substr(16 + data_packet_start, 2), nullptr, 16)) << 8 )| (uint8_t)std::stoi(buffer_s.substr(18 + data_packet_start, 2), nullptr, 16);
            my_motor_motion.motor_location_lb = (((uint8_t)std::stoi(buffer_s.substr(24 + data_packet_start, 2), nullptr, 16)) << 24 )| (((uint8_t)std::stoi(buffer_s.substr(26 + data_packet_start, 2), nullptr, 16)) << 16 ) | (((uint8_t)std::stoi(buffer_s.substr(28 + data_packet_start, 2), nullptr, 16)) << 8 ) | ((uint8_t)std::stoi(buffer_s.substr(30 + data_packet_start, 2), nullptr, 16));
          }
         
          else if(buffer_s.find("0221")-data_packet_start == 12)
          {
            cmd_feedback.ctrl_cmd_velocity = (((uint8_t)std::stoi(buffer_s.substr(16 + data_packet_start, 2), nullptr, 16)) << 8 )| (uint8_t)std::stoi(buffer_s.substr(18 + data_packet_start, 2), nullptr, 16);
            cmd_feedback.ctrl_cmd_steering = (((uint8_t)std::stoi(buffer_s.substr(20 + data_packet_start, 2), nullptr, 16)) << 8 )| (uint8_t)std::stoi(buffer_s.substr(22 + data_packet_start, 2), nullptr, 16);
            //标定反馈与输入
            cmd_feedback.ctrl_cmd_velocity = static_cast<int16_t>(cmd_feedback.ctrl_cmd_velocity * 1.51);
            cmd_feedback.ctrl_cmd_steering = static_cast<int16_t>(cmd_feedback.ctrl_cmd_steering * 2.38); 
          }
         
        }
        
        buffer_s.clear();
    }

    void
    CODY::sub_command(const can_interfaces::CtrlCmd::ConstPtr& msg)
    {
        my_cmd.ctrl_cmd_velocity = msg->ctrl_cmd_velocity;
        
        my_cmd.ctrl_cmd_steering = msg->ctrl_cmd_steering;
    
    }

    void
    CODY::publish_topic()
    {
        cmd_feed_pub.publish(cmd_feedback);
        odom_feed_pub.publish(odom_feedback);
        motor_feed_pub.publish(my_motor_motion);
        my_pose.header.stamp = ros::Time::now();
        my_pose.header.frame_id = "map";
        pose_feed_pub.publish(my_pose);
    }

    void
    CODY::spst_control()
    {
      std::string hexData = "AA 00 00 08 00 00 01 11 00 00 00 00 00 00 00 00";
      auto bytes0 = hexStringToBytes(hexData);

      std::cout << "my_cmd is " << my_cmd.ctrl_cmd_velocity << std::endl;
      uint8_t highByte_vel = (static_cast<uint16_t>(my_cmd.ctrl_cmd_velocity) >> 8) & 0xFF;
      uint8_t lowByte_vel = static_cast<uint16_t>(my_cmd.ctrl_cmd_velocity) & 0xFF;
      
      
      bytes0[8] = highByte_vel;
      bytes0[9] = lowByte_vel;

       std::cout << "Bytes: ";
       for (int i = 0 ; i < 16; i++) {
           std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(bytes0[i]) << " ";
       }
       std::cout << std::endl;


      uint8_t highByte_angle = (static_cast<uint16_t>(my_cmd.ctrl_cmd_steering) >> 8) & 0xFF;
      uint8_t lowByte_angle = static_cast<uint16_t>(my_cmd.ctrl_cmd_steering) & 0xFF;
      
      bytes0[10] = highByte_angle;
      bytes0[11] = lowByte_angle;
      sp.writeData(bytes0, 16);

    }

}
