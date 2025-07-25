/******************************************************************************
 * This file is part of lslidar_driver.
 *
 * Copyright 2022 LeiShen Intelligent Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef INPUT_HPP
#define INPUT_HPP

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <signal.h>
#include <sensor_msgs/TimeReference.h>
#include "lslidar_msgs/LslidarPacket.h"


namespace lslidar_driver {
    static uint16_t MSOP_DATA_PORT_NUMBER = 2368;   // lslidar default data port on PC
    static uint16_t DIFOP_DATA_PORT_NUMBER = 2369;  // lslidar default difop data port on PC
/**
 *  从在线的网络数据或离线的网络抓包数据（pcap文件）中提取出lidar的原始数据，即packet数据包
 * @brief The Input class,
     *
     * @param private_nh  一个NodeHandled,用于通过节点传递参数
     * @param port
     * @returns 0 if successful,
     *          -1 if end of file
     *          >0 if incomplete packet (is this possible?)
 */
    class Input {
    public:
        Input(ros::NodeHandle private_nh, uint16_t port , int packet_size);

        virtual ~Input() {
        }

        virtual int getPacket(lslidar_msgs::LslidarPacketPtr &pkt) = 0;

        virtual void sendPacket(const unsigned char *data, size_t length) = 0;

    protected:
        ros::NodeHandle private_nh_;
        uint16_t port_;
        std::string devip_str_;
        int difop_port_;
        int cur_rpm_;
        int return_mode_;
        bool npkt_update_flag_;
        bool add_multicast;
        std::string group_ip;
        int packet_size_;
    };

/** @brief Live lslidar input from socket. */
    class InputSocket : public Input {
    public:
        InputSocket(ros::NodeHandle private_nh, uint16_t port = MSOP_DATA_PORT_NUMBER,int packet_size = 1212);

        virtual ~InputSocket();

        virtual int getPacket(lslidar_msgs::LslidarPacketPtr &pkt);

        virtual void sendPacket(const unsigned char *data, size_t length);

    private:
        int sockfd_;
        in_addr devip_;
    };

/** @brief lslidar input from PCAP dump file.
   *
   * Dump files can be grabbed by libpcap
   */
    class InputPCAP : public Input {
    public:
        InputPCAP(ros::NodeHandle private_nh, uint16_t port = MSOP_DATA_PORT_NUMBER, int packet_size = 1212, double packet_rate = 0.0,
                  std::string filename = "", bool read_once = false, bool read_fast = false, double repeat_delay = 0.0);

        virtual ~InputPCAP();

        virtual int getPacket(lslidar_msgs::LslidarPacketPtr &pkt);

        virtual void sendPacket(const unsigned char *data, size_t length);

    private:
        ros::Rate packet_rate_;
        std::string filename_;
        pcap_t *pcap_;
        bpf_program pcap_packet_filter_;
        char errbuf_[PCAP_ERRBUF_SIZE];
        bool empty_;
        bool read_once_;
        bool read_fast_;
        double repeat_delay_;
    };
}

#endif  // INPUT_HPP
