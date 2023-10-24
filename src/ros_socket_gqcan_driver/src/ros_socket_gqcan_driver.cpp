#include "ros_socket_gqcan_driver.h"

#define BUFF_SIZE 15

static uint64_t u64_from_can_msg(const uint8_t m[8])
{
    return  ((uint64_t)m[7] << 56) |
            ((uint64_t)m[6] << 48) |
            ((uint64_t)m[5] << 40) |
            ((uint64_t)m[4] << 32) |
            ((uint64_t)m[3] << 24) |
            ((uint64_t)m[2] << 16) |
            ((uint64_t)m[1] << 8) |
            ((uint64_t)m[0] << 0);
}

void u64_to_can_msg(const uint64_t u, std::vector<uint8_t>& m) {

    m.clear();
    for(int i =0; i<=56; i+=8)
    {
      m.push_back(u >> i);
    }
}

void operator>>(const YAML::Node &node, CANMsg &msg)
{
  msg.name = node["name"].as<std::string>();
  msg.channel = (uint8_t)node["channel"].as<int>();
  msg.can_id = (uint16_t)node["can_id"].as<int>();
  msg.can_dlc = (uint8_t)node["can_dlc"].as<int>();
  msg.can_data = node["can_data"].as<std::vector<uint8_t>>();
}

CANInfo loadYMLFile(std::string ymlpath)
{
  CANInfo info;
  try
  {
    YAML::Node node = YAML::LoadFile(ymlpath);
    CANMsg msg;
    for (int i = 0; i < (int)node.size(); i++)
    {
      node[i] >> msg;
      // std::cout << msg.name << std::endl;
      info.msgs.push_back(msg);
    }
  }
  catch (YAML::Exception &e)
  {
    std::cerr << e.what() << std::endl;
  }
  return info;
}

SocketCanDriver::SocketCanDriver()
    : nh_(""), pnh_("~")
{
  // ROS
  // deal messages from thread
  socket_can_rx_pub_ = nh_.advertise<can_msgs::CANFrame>("CAN_rx_thread", 1);
  parse_can_rx_sub_ = nh_.subscribe("CAN_rx_thread", 1, &SocketCanDriver::parseRxCallback, this,
                                    ros::TransportHints().tcpNoDelay(true));

  // deal messages with CAN_parse module and PNC module
  socket_can_status_pub_ = nh_.advertise<can_msgs::gq_car>("GQ_Car_Status", 1000);

  // parameters
  pnh_.param("max_ch_num", max_ch_num_, 2);
  pnh_.param("bitrate", bitrate_, 500000);
  pnh_.param("virtual", virtual_, false);
  pnh_.param("can_rx_setting", can_rx_setting_, std::string("can_rx.yaml"));

  rx_info_.msgs.clear();
  std::string can_rx_file;
  can_rx_file = ros::package::getPath("ros_socket_gqcan_driver") + "/config/" + can_rx_setting_;
  rx_info_ = loadYMLFile(can_rx_file);

  if(rx_info_.msgs.empty())
  {
    ROS_ERROR("Load CAN_Rx yaml failed, rx_info msgs is empty!");
    ros::shutdown();
    return;
  }

  receive_thread_list_.clear();
  socket_mtx_ = new std::mutex[max_ch_num_];

  // socket can initialize
  for (int i = 0; i < max_ch_num_; ++i)
  {
    try
    {
      // Open can socket
      int s;
      if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
      {
        ROS_ERROR("socket open error");
        throw(socket_can_exception::OPEN_FAIL);
      }
      ROS_INFO("Open %d channel", i);

      int rcvbuf_size = BUFF_SIZE;
      if (setsockopt(s, SOL_SOCKET, SO_RCVBUF,
                     &rcvbuf_size, sizeof(rcvbuf_size)) < 0)
      {
        ROS_ERROR("setsockopt SO_RCVBUF");
        throw(socket_can_exception::CONFIG_FAIL);
      }

      struct sockaddr_can addr;
      struct ifreq ifr;
      std::string bus_name = "can";
      if (virtual_)
      {
        bus_name = "vcan";
      }
      std::string can_channel_name = bus_name + std::to_string(i);
      sprintf(ifr.ifr_name, "%s", can_channel_name.c_str());

      ifr.ifr_name[IFNAMSIZ - 1] = '\0';
      ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
      if (!ifr.ifr_ifindex)
      {
        ROS_ERROR("if_nametoindex %s error", ifr.ifr_name);
        throw(socket_can_exception::CONFIG_FAIL);
      }

      ioctl(s, SIOCGIFINDEX, &ifr);

      memset(&addr, 0, sizeof(addr));
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;

      if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
      {
        ROS_ERROR("bind error");
        throw(socket_can_exception::BIND_FAIL);
      }

      ROS_INFO("Bind %s", ifr.ifr_name);

      // Save
      socket_hnd_.emplace_back(s);
      addr_list_.emplace_back(addr);

      // Thread
      receive_thread_list_.emplace_back(std::thread([this, i]() { this->can_receive_thread(i); }));
      ROS_INFO("Make %d thread", i);
    }

    catch (const socket_can_exception &ex)
    {
      switch (ex)
      {
      case socket_can_exception::OPEN_FAIL:
        ROS_ERROR("CAN channel %d : Device open error.", i);
        break;
      case socket_can_exception::CONFIG_FAIL:
        ROS_ERROR("CAN channel %d : Device setting error.", i);
        break;
      case socket_can_exception::BIND_FAIL:
        ROS_ERROR("CAN channel %d : Device setting error.", i);
        break;
      case socket_can_exception::RECV_ERR:
        ROS_ERROR("CAN channel %d : Device receive error.", i);
        break;
      case socket_can_exception::NO_MSG:
        ROS_ERROR("CAN channel %d : Device no message error.", i);
        break;
      case socket_can_exception::TIMEOUT:
        ROS_ERROR("CAN channel %d : Device timeout error.", i);
        break;
      }
      ros::shutdown();
      return;
    }
    catch (...)
    {
      ROS_ERROR("CAN channel %d : UNKNOWN error.", i);
      ros::shutdown();
      return;
    }
  }

  ros::Time stamp;
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
  }

} // end SocketCanDriver()

/*--------------------------------------------------------------------
 * ~SocketCanDriver()
 * Destructor.
 *------------------------------------------------------------------*/
SocketCanDriver::~SocketCanDriver()
{
  for (int i = 0; i < max_ch_num_; ++i)
  {
    if ((int)receive_thread_list_.size() > i)
    {
      receive_thread_list_[i].join();
      int hnd = socket_hnd_[i];
      close(hnd);
    }
  }
  delete socket_mtx_;
} // end ~SocketCanDriver()


/*--------------------------------------------------------------------
 * parseRxCallback()
 * Callback function for parse CAN msg to resolve chassis status.
 *------------------------------------------------------------------*/
// callback

void SocketCanDriver::parseRxCallback(const can_msgs::CANFrame::ConstPtr& can_frame_msg)
{
  // ROS_INFO("parseRx_callback!");
  uint64_t can_message_u64;
  can_message_u64 = u64_from_can_msg(& can_frame_msg -> can_data[0]);
  unpack_message(&can_GQ_object_, can_frame_msg ->can_id, can_message_u64, can_frame_msg -> can_dlc, 1);

  if(can_frame_msg ->can_id == 0x171)
  {

    double id171_double_value;

    decode_can_0x171_DCU_RosSpdAct(&can_GQ_object_, &id171_double_value);
    gq_status_msgs_.motor_speed = id171_double_value;

    decode_can_0x171_DCU_IdcAct(&can_GQ_object_, &id171_double_value);
    gq_status_msgs_.battery_current = id171_double_value;

    decode_can_0x171_DCU_TorqAct(&can_GQ_object_, &id171_double_value);
    gq_status_msgs_.motor_torque = id171_double_value;

    uint16_t id171_int_value;

    decode_can_0x171_DCU_UdcAct(&can_GQ_object_, &id171_int_value);
    gq_status_msgs_. battery_voltage = id171_int_value;

  }else if(can_frame_msg ->can_id == 0x176)
  {
    double id176_double_value;

    decode_can_0x176_VCU_EMS_AccPedalActPst(&can_GQ_object_, &id176_double_value);
    gq_status_msgs_.gas_pedal = id176_double_value;

  }else if(can_frame_msg ->can_id == 0x260)
  {
    double id260_double_value;

    decode_can_0x260_BCS_VehSpd(&can_GQ_object_, &id260_double_value);
    gq_status_msgs_.car_speed = id260_double_value;

  }else if(can_frame_msg ->can_id == 0x26a)
  {
    double id26a_double_value;

    decode_can_0x26a_BCS_ActVehLongAccel(&can_GQ_object_, &id26a_double_value);
    gq_status_msgs_.car_acc = id26a_double_value;

  }else if(can_frame_msg ->can_id == 0x283)
  {
    double id283_double_value;

    decode_can_0x283_EBB_brkPedPst(&can_GQ_object_, &id283_double_value);
    gq_status_msgs_.break_pedal = id283_double_value;

  }else if(can_frame_msg ->can_id == 0x2ab)
  {
    uint8_t id2ab_int_value;

    decode_can_0x2ab_VCU_CrntGearLvl(&can_GQ_object_, &id2ab_int_value);
    gq_status_msgs_.gear_leval = id2ab_int_value;

  }else if(can_frame_msg ->can_id == 0x2ad)
  {
    double id2ad_double_value;

    decode_can_0x2ad_BMS_BattSocDisp(&can_GQ_object_, &id2ad_double_value);
    gq_status_msgs_.battery_soc = id2ad_double_value;

  }else if(can_frame_msg ->can_id == 0x360)
  {
    uint8_t id360_int_value;

    decode_can_0x360_VCU_VehRdySt(&can_GQ_object_, &id360_int_value);
    gq_status_msgs_.ready_status = id360_int_value;

  }

  gq_status_msgs_.header.stamp = ros::Time::now();

  socket_can_status_pub_.publish(gq_status_msgs_);
  ROS_INFO("GQ Car Msgs Sending......");

}

/*--------------------------------------------------------------------
 * can_receive_thread()
 * Thread for receive che channel CAN raw frame using that Socket_CAN recvmmsg() function.
 *------------------------------------------------------------------*/
// thread

void SocketCanDriver::can_receive_thread(unsigned char ch_num)
{
  struct sockaddr_can addr = addr_list_[ch_num];
  ros::Time stamp;
  int retval;
  struct mmsghdr msgs[BUFF_SIZE];
  struct iovec iovecs[BUFF_SIZE];
  struct can_frame frames[BUFF_SIZE];
  struct timespec timeout;
  memset(msgs, 0, sizeof(msgs));

  // ROS_INFO("Start handle %d receive thread", ch_num);

  for (int i = 0; i < BUFF_SIZE; i++)
  {
		struct iovec *iovec = &iovecs[i];
		struct mmsghdr *msg = &msgs[i];

		msg->msg_hdr.msg_iov = iovec;
		msg->msg_hdr.msg_iovlen = 1;

		iovec->iov_base = &frames[i];
		iovec->iov_len = sizeof(can_frame);
  }

  timeout.tv_sec = 0;
  timeout.tv_nsec = 0;

  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    // Read CAN Data
    try
    {
      // const double recev_s_time = ros::WallTime::now().toSec();
      // 一次读取BUFF_SIZE个msg, 存储到msgs;
      //int retval = recvmmsg(socket_hnd_[ch_num], msgs, BUFF_SIZE, MSG_DONTWAIT, NULL);
      int retval = recvmmsg(socket_hnd_[ch_num], msgs, BUFF_SIZE, 0, NULL);

      // ROS_INFO("receive msg time: %.8f", ros::WallTime::now().toSec() - recev_s_time);

      if (retval < 0)
      {
        ROS_WARN_DELAYED_THROTTLE(10, "read error %d", ch_num);
        continue;
      }
      // ROS_WARN_DELAYED_THROTTLE(1, "In channel %d, the recvmmsg number is %d!", ch_num, retval);
      // 对读取的n数量个msg进行处理
      for (int i = 0; i < retval; i++)
      {
        char buf[CAN_MAX_DLEN];
        int id = frames[i].can_id;
        int dlc = frames[i].can_dlc;
        stamp = ros::Time::now();

        // Processing data
        for (int j = 0; j < (int)rx_info_.msgs.size(); ++j)
        {
          if (rx_info_.msgs[j].can_id == id &&
              rx_info_.msgs[j].channel == ch_num &&
              rx_info_.msgs[j].can_dlc == dlc)
          {
            can_msgs::CANFrame out_frame;
            out_frame.can_msg_name = rx_info_.msgs[j].name;
            out_frame.can_channel = ch_num;
            out_frame.header.stamp = stamp;
            out_frame.header.frame_id = "/can/data";
            out_frame.can_id = id;
            out_frame.can_dlc = dlc;
            for (int k = 0; k < (int)dlc; ++k)
            {
              out_frame.can_data.push_back(frames[i].data[k]);
            }
            socket_can_rx_pub_.publish(out_frame);
            // ROS_WARN_DELAYED_THROTTLE(1, "Receive channel %d; Socket publish %0X frame!", ch_num, id);
            // ROS_WARN("Receive channel %d; Socket publish %0X frame!", ch_num, id);
          }
        }
      }
      // ROS_INFO("process msg total time: %.8f", ros::WallTime::now().toSec() - recev_s_time);
    }
    catch (...)
    {
      ROS_ERROR("CAN ch%d: Unexpected Error.", ch_num);
      ros::shutdown();
    }

    ros::spinOnce();
    loop_rate.sleep();

  }
  ROS_WARN("Exit receive thread %d", ch_num);
}

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/
int32_t main(int32_t argc, char **argv)
{
  // Setup ROS.
  ros::init(argc, argv, "ros_socket_can_driver_node");

  SocketCanDriver node;

  ros::spin();
  std::cerr << "\nros_socket_can_driver_node: Exiting...\n";
  ros::shutdown();
  return (EXIT_SUCCESS);
} // end main()
