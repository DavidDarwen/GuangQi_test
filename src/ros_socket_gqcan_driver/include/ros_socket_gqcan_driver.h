
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include <ros/rate.h>
#include <vector>
#include <string>
#include <thread>
#include <mutex>

#include <boost/thread.hpp>

#include <yaml-cpp/yaml.h>
#include <fstream>

// msgs
#include <can_msgs/CANFrame.h>
#include <can_msgs/gq_car.h>

#include "dbc_file/CAN_GQ.h"

enum class socket_can_exception
{
  OPEN_FAIL,
  CONFIG_FAIL,
  BIND_FAIL,
  RECV_ERR,
  NO_MSG,
  TIMEOUT,
};

typedef struct CANMsg {
  std::string name;
  uint8_t channel;
  uint16_t can_id;
  uint8_t can_dlc;
	std::vector<uint8_t> can_data;
} CANMsg;

typedef struct CANInfo{
    std::vector<CANMsg> msgs;
} CANInfo;

class SocketCanDriver
{
private:
	ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

	//! Publisher
  // Thread inner pub
	ros::Publisher socket_can_rx_pub_;

  // Subscriber
  // Subscibe the messages from rx thread
  ros::Subscriber parse_can_rx_sub_;

  // Publisher pub the status messages
  ros::Publisher socket_can_status_pub_;

	std::vector<std::thread> receive_thread_list_;
	std::vector<std::thread> send_thread_list_;
	std::vector<int> socket_hnd_;
	
  std::vector<struct sockaddr_can> addr_list_;
  int max_ch_num_;
  int bitrate_;
  bool virtual_;
  std::string can_tx_setting_;
  std::string can_rx_setting_;

  CANInfo rx_info_;
  std::mutex *socket_mtx_;



	// callback functions <parse the PNC control command to CAN_frame data>
  void parseRxCallback(const can_msgs::CANFrame::ConstPtr& can_frame_msg);

	// thread
	void can_send_thread(unsigned char ch_num);
	void can_receive_thread(unsigned char ch_num);

  can_obj_can_gq_h_t can_GQ_object_;
  // can_obj_canb_h_t canB_object_;

  can_msgs::gq_car gq_status_msgs_;


public:
  //! Constructor.
  SocketCanDriver();

  //! Destructor.
  ~SocketCanDriver();
};
