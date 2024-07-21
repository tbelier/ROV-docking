// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <vector>

#include <time.h>

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#ifdef _WIN32
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#endif

#include "an_packet_protocol.h"
#include "subsonus_packets.h"

#define RADIANS_TO_DEGREES (180.0/M_PI)


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "subsonus_pkg/msg/subsonus_raw_sensors_packet.hpp"
#include <Eigen/Dense>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;


/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
uint32_t parseIPV4string(char* ipAddress)
{
	unsigned int ipbytes[4];
	sscanf(ipAddress, "%uhh.%uhh.%uhh.%uhh", &ipbytes[3], &ipbytes[2], &ipbytes[1], &ipbytes[0]);
	return ipbytes[0] | ipbytes[1] << 8 | ipbytes[2] << 16 | ipbytes[3] << 24;
}

int an_packet_transmit(an_packet_t *an_packet)
{
	an_packet_encode(an_packet);
	return 0; //SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
}

/*
 * This is an example of sending a configuration packet to Spatial.
 *
 * 1. First declare the structure for the packet, in this case filter_options_packet_t.
 * 2. Set all the fields of the packet structure
 * 3. Encode the packet structure into an an_packet_t using the appropriate helper function
 * 4. Send the packet
 * 5. Free the packet
 */

void set_network_options()
{
	an_packet_t *an_packet = an_packet_allocate(30, packet_id_network_settings);

	network_settings_packet_t network_settings_packet;

	// initialise the structure by setting all the fields to zero
	memset(&network_settings_packet, 0, sizeof(network_settings_packet_t));

	network_settings_packet.dhcp_mode_flags.b.dhcp_enabled = 1;
	network_settings_packet.dhcp_mode_flags.b.automatic_dns = 1;
	network_settings_packet.dhcp_mode_flags.b.link_mode = link_auto;
	network_settings_packet.permanent = 1;

	network_settings_packet.static_dns_server = (uint32_t) parseIPV4string("0.0.0.0");  // usually the network modem: e.g. 192.168.1.1
	network_settings_packet.static_gateway = (uint32_t) parseIPV4string("0.0.0.0");     // usually the network modem: e.g. 192.168.1.1
	network_settings_packet.static_ip_address = (uint32_t) parseIPV4string("0.0.0.0");  // e.g. 192.168.1.20
	network_settings_packet.static_netmask = (uint32_t) parseIPV4string("0.0.0.0");     // e.g. 255.255.255.0

	encode_network_settings_packet(an_packet, &network_settings_packet);
	an_packet_encode(an_packet);

	an_packet_transmit(an_packet);

	an_packet_free(&an_packet);
}


int tcp_socket;
struct sockaddr_in serveraddr;
struct hostent *server;


class NodeData : public rclcpp::Node
{
public:
  
  clock_t begin = clock();
  int id=0;
  int variable_test = 0;

  subsonus_pkg::msg::SubsonusRawSensorsPacket msg_raw_sensors_packet = subsonus_pkg::msg::SubsonusRawSensorsPacket();

  NodeData()
  : Node("nodeRawdata_ROV"), count_(0)
  {

    unsigned int bytes_received = 0;
    char *hostname;
    hostname = "192.168.2.200";
    int port = 20000; // Port pour les données brutes magnétiques et accéléromètres

    tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
    if(tcp_socket < 0)
    {
      printf("Could not open TCP socket\n");
      exit(EXIT_FAILURE);
    }

    // Find the address of the host
    server = gethostbyname("192.168.2.200");
    if(server == NULL)
    {
      printf("Could not find host %s\n", hostname);
      exit(EXIT_FAILURE);
    }
    else{
      printf("ok host\n");
    }

    // Set the server's address
    memset((char *) &serveraddr, 0, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    memcpy((char *) &serveraddr.sin_addr.s_addr, (char *) server->h_addr_list[0], server->h_length);
    serveraddr.sin_port = htons(port);
    if(connect(tcp_socket, (struct sockaddr *) &serveraddr, sizeof(serveraddr)) < 0)
    {
      printf("Could not connect to server\n");
    }else{
      printf("ok socket\n");
    }

    {
      int flush_length = 0;
      while(1)
      {
        struct timeval t;
        fd_set readfds;
        t.tv_sec = 0;
        t.tv_usec = 50000;
        unsigned char buf[1024];
        FD_ZERO(&readfds);
        FD_SET(tcp_socket, &readfds);
        select(tcp_socket + 1, &readfds, NULL, NULL, &t);
        if(FD_ISSET(tcp_socket, &readfds))
        {
          flush_length = recv(tcp_socket, buf, sizeof(buf), 0);
          if(flush_length < 100)
          {
            break;
          }
        }
        else
        {
          break;
        }
      }
    }

    printf("Encode Network Settings Packet:\n");
    set_network_options();

    publisher_raw_sensors_packet =  this->create_publisher<subsonus_pkg::msg::SubsonusRawSensorsPacket>("subsonus/raw_sensors_packet_ROV", 1000);

    timer_ = this->create_wall_timer(
      10ms, std::bind(&NodeData::timer_callback, this));
  }

private:
  void timer_callback()
  {  
    id++;

    unsigned int bytes_received = 0;
    struct timeval t;
    fd_set readfds;
    t.tv_sec = 0;
    t.tv_usec = 10000;

    an_decoder_t an_decoder;
    an_packet_t *an_packet;

    an_decoder_initialise(&an_decoder);

    subsonus_raw_sensors_packet_t subsonus_raw_sensors_packet;
    subsonus_remote_raw_sensors_packet_t subsonus_remote_raw_sensors_packet;
    FD_ZERO(&readfds);
		FD_SET(tcp_socket, &readfds);
		select(tcp_socket + 1, &readfds, NULL, NULL, &t);
		if(FD_ISSET(tcp_socket, &readfds))
		{

#ifdef _WIN32
			bytes_received = recv(tcp_socket,an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder),0);
#else
			bytes_received = recv(tcp_socket, an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder), MSG_DONTWAIT);
#endif

			if(bytes_received > 0)
			{
				/* increment the decode buffer length by the number of bytes received */
				an_decoder_increment(&an_decoder, bytes_received);

				/* decode all the packets in the buffer */
				while((an_packet = an_packet_decode_dynamic(&an_decoder)) != NULL)
				{
          // printf("test %u\n",an_packet->id);
					if(an_packet->id == packet_id_subsonus_raw_sensors_packet)
					{
						if(decode_subsonus_raw_sensors_packet(&subsonus_raw_sensors_packet, an_packet) == 0)
						{
              msg_raw_sensors_packet.accelerometer_x = subsonus_raw_sensors_packet.accelerometers[0];
              msg_raw_sensors_packet.accelerometer_y = subsonus_raw_sensors_packet.accelerometers[1];
              msg_raw_sensors_packet.accelerometer_z = subsonus_raw_sensors_packet.accelerometers[2];
              msg_raw_sensors_packet.gyroscope_x = subsonus_raw_sensors_packet.gyroscopes[0];
              msg_raw_sensors_packet.gyroscope_y = subsonus_raw_sensors_packet.gyroscopes[1];
              msg_raw_sensors_packet.gyroscope_z = subsonus_raw_sensors_packet.gyroscopes[2];
              msg_raw_sensors_packet.magnetometer_x = subsonus_raw_sensors_packet.magnetometers[0];
              msg_raw_sensors_packet.magnetometer_y = subsonus_raw_sensors_packet.magnetometers[1];
              msg_raw_sensors_packet.magnetometer_z = subsonus_raw_sensors_packet.magnetometers[2];
              msg_raw_sensors_packet.internal_temperature = subsonus_raw_sensors_packet.internal_temperature;
              msg_raw_sensors_packet.pressure_depth = subsonus_raw_sensors_packet.pressure_depth;
              msg_raw_sensors_packet.water_temperature = subsonus_raw_sensors_packet.water_temperature;
              msg_raw_sensors_packet.velocity_of_sound = subsonus_raw_sensors_packet.velocity_of_sound;


              // Calcul du cap
              Eigen::MatrixXd A(3, 3);
              A << 1.074043508698078409e+00, -1.825169088465639078e-02, -2.506236595745721663e-01,
                  -1.825169088465639078e-02, 9.203150980722383245e-01, 9.558293574412760063e-02,
                  -2.506236595745722218e-01, 9.558293574412757287e-02, 1.079564619015944915e+00;

              Eigen::VectorXd b(3);
              b << -1.782746475458791906e+02,
                    1.906317241941198972e+02,
                    6.746973582435566641e+02;

              Eigen::VectorXd X(3);
              X << subsonus_raw_sensors_packet.magnetometers[0],
                   subsonus_raw_sensors_packet.magnetometers[1],
                   subsonus_raw_sensors_packet.magnetometers[2];

              Eigen::VectorXd g(3);
              g << subsonus_raw_sensors_packet.accelerometers[0],
                   subsonus_raw_sensors_packet.accelerometers[1],
                   subsonus_raw_sensors_packet.accelerometers[2];
              Eigen::Vector3d g_normalized = g.normalized();

              Eigen::VectorXd X_nord(3);
              X_nord << 165,
                        165,
                        1050;
              Eigen::VectorXd X_nord_calib = A*(X_nord-b);

              Eigen::VectorXd X_nord_proj = X_nord - X_nord.dot(g_normalized)*g_normalized;
              Eigen::VectorXd X_nord_proj_calib = X_nord_calib - X_nord_calib.dot(g_normalized)*g_normalized;

              Eigen::VectorXd X_nord_proj_normalized = X_nord_proj.normalized();
              Eigen::VectorXd X_nord_proj_calib_normalized = X_nord_proj_calib.normalized();

              // Eigen::VectorXd X_cor = A.inverse()*(X-b);
              Eigen::VectorXd X_cor = A*(X-b);

              Eigen::VectorXd X_proj = X - X.dot(g_normalized)*g_normalized;
              Eigen::VectorXd X_proj_calib = X_cor - X_cor.dot(g_normalized)*g_normalized;
             
              Eigen::VectorXd X_proj_normalized = X_proj.normalized();
              Eigen::VectorXd X_proj_calib_normalized = X_proj_calib.normalized();
                            

              double cap_calib = std::acos(X_nord_proj_calib_normalized.dot(X_proj_calib_normalized));
              if (X_proj_calib_normalized(1) > 0) 
              {
                 cap_calib = -cap_calib;
              }
              double cap = std::acos(X_nord_proj_normalized.dot(X_proj_normalized));
              if (X_proj_normalized(1) > 0) 
              {
                 cap = -cap;
              }

              msg_raw_sensors_packet.cap_calib = 180*cap_calib/M_PI;
              msg_raw_sensors_packet.cap = 180*cap/M_PI;

              publisher_raw_sensors_packet->publish(msg_raw_sensors_packet);
						}
					}

					/* Ensure that you free the an_packet when your done with it or you will leak memory */
					an_packet_free(&an_packet);
				}
			}
		}   
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<subsonus_pkg::msg::SubsonusRawSensorsPacket>::SharedPtr publisher_raw_sensors_packet;

  size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NodeData>());
  rclcpp::shutdown();
  return 0;
}
