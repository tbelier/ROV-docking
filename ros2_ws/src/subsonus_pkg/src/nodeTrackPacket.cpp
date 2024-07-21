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
#include "subsonus_pkg/msg/subsonus_remote_track_packet.hpp"

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

  subsonus_pkg::msg::SubsonusRemoteTrackPacket msg_track_packet = subsonus_pkg::msg::SubsonusRemoteTrackPacket();
  subsonus_pkg::msg::SubsonusRemoteTrackPacket msg_track_packet_filtered = subsonus_pkg::msg::SubsonusRemoteTrackPacket();

  const std::vector<double>::size_type maxSize = 3;
  std::vector<double> filtered_range;
  std::vector<double> filtered_azimuth;
  std::vector<double> filtered_elevation;
  std::vector<double> filtered_pos_x;
  std::vector<double> filtered_pos_y;
  std::vector<double> filtered_pos_z;

  NodeData()
  : Node("nodeTrackPacket"), count_(0)
  {

    unsigned int bytes_received = 0;
    char *hostname;
    hostname = "192.168.2.100";
    int port = 19000; // Port pour les données brutes magnétiques et accéléromètres

    tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
    if(tcp_socket < 0)
    {
      printf("Could not open TCP socket\n");
      exit(EXIT_FAILURE);
    }

    // Find the address of the host
    server = gethostbyname("192.168.2.100");
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

    printf("Encode Network Settings Packet:\n");
    set_network_options();

    publisher_track_packet =  this->create_publisher<subsonus_pkg::msg::SubsonusRemoteTrackPacket>("subsonus/track_packet", 1);
	publisher_track_packet_filtered =  this->create_publisher<subsonus_pkg::msg::SubsonusRemoteTrackPacket>("subsonus/track_packet_filtered", 1);

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

    subsonus_track_packet_t subsonus_track_packet;
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
					if(an_packet->id == packet_id_subsonus_track)
					{
						if(decode_subsonus_track_packet(&subsonus_track_packet, an_packet) == 0)
						{
						  // Données brutes
			              msg_track_packet.device_address = subsonus_track_packet.device_address;
			              msg_track_packet.tracking_status = subsonus_track_packet.tracking_status.r;
			              msg_track_packet.local_system_status = subsonus_track_packet.observer_system_status.r;
			              msg_track_packet.local_filter_status = subsonus_track_packet.observer_filter_status.r;
			              msg_track_packet.data_valid_flags = subsonus_track_packet.data_valid.r;
			              msg_track_packet.local_unix_time_stamp_seconds = subsonus_track_packet.observer_unix_time_seconds;
			              msg_track_packet.local_microseconds = subsonus_track_packet.observer_microseconds;
			              msg_track_packet.local_latitude = subsonus_track_packet.observer_latitude;
			              msg_track_packet.local_longitude = subsonus_track_packet.observer_longitude;
			              msg_track_packet.local_height = subsonus_track_packet.observer_height;
			              msg_track_packet.local_velocity_north = subsonus_track_packet.observer_velocity[0];
			              msg_track_packet.local_velocity_east = subsonus_track_packet.observer_velocity[1];
			              msg_track_packet.local_velocity_down = subsonus_track_packet.observer_velocity[2];
			              msg_track_packet.local_roll = subsonus_track_packet.observer_orientation[0];
			              msg_track_packet.local_pitch = subsonus_track_packet.observer_orientation[1];
			              msg_track_packet.local_heading = subsonus_track_packet.observer_orientation[2];
			              msg_track_packet.local_latitude_standard_deviation = subsonus_track_packet.observer_position_standard_deviation[0];
			              msg_track_packet.local_longitude_standard_deviation = subsonus_track_packet.observer_position_standard_deviation[1];
			              msg_track_packet.local_height_standard_deviation = subsonus_track_packet.observer_position_standard_deviation[2];
			              msg_track_packet.local_roll_standard_deviation = subsonus_track_packet.observer_orientation_standard_deviation[0];
			              msg_track_packet.local_pitch_standard_deviation = subsonus_track_packet.observer_orientation_standard_deviation[1];
			              msg_track_packet.local_heading_standard_deviation = subsonus_track_packet.observer_orientation_standard_deviation[2];
			              msg_track_packet.local_depth = subsonus_track_packet.observer_depth;
			              msg_track_packet.remote_age_in_microseconds = subsonus_track_packet.age_microseconds;
			              msg_track_packet.remote_range = subsonus_track_packet.range;
			              msg_track_packet.remote_azimuth = subsonus_track_packet.azimuth;
			              msg_track_packet.remote_elevation = subsonus_track_packet.elevation;
			              msg_track_packet.remote_position_raw_x = subsonus_track_packet.raw_position[0];
			              msg_track_packet.remote_position_raw_y = subsonus_track_packet.raw_position[1];
			              msg_track_packet.remote_position_raw_z = subsonus_track_packet.raw_position[2];
			              msg_track_packet.remote_position_x = subsonus_track_packet.corrected_position[0];
			              msg_track_packet.remote_position_y = subsonus_track_packet.corrected_position[1];
			              msg_track_packet.remote_position_z = subsonus_track_packet.corrected_position[2];
			              msg_track_packet.remote_north = subsonus_track_packet.ned_position[0];
			              msg_track_packet.remote_east = subsonus_track_packet.ned_position[1];
			              msg_track_packet.remote_down = subsonus_track_packet.ned_position[2];
			              msg_track_packet.remote_latitude = subsonus_track_packet.latitude;
			              msg_track_packet.remote_longitude = subsonus_track_packet.longitude;
			              msg_track_packet.remote_height = subsonus_track_packet.height;
			              msg_track_packet.remote_range_standard_deviation = subsonus_track_packet.range_standard_deviation;
			              msg_track_packet.remote_azimuth_standard_deviation = subsonus_track_packet.azimuth_standard_deviation;
			              msg_track_packet.remote_elevation_standard_deviation = subsonus_track_packet.elevation_standard_deviation;
			              msg_track_packet.remote_latitude_standard_deviation = subsonus_track_packet.latitude_standard_deviation;
			              msg_track_packet.remote_longitude_standard_deviation = subsonus_track_packet.longitude_standard_deviation;
			              msg_track_packet.remote_height_standard_deviation = subsonus_track_packet.height_standard_deviation;
			              msg_track_packet.remote_depth = subsonus_track_packet.depth;

			              publisher_track_packet->publish(msg_track_packet);


			              // Filtrage des données intéressantes
						  double median_pos_x;
			              double median_pos_y;
			              double median_pos_z;
			              double median_range;
			              double median_azimuth;

			              if (filtered_pos_x.size() < maxSize){
			                filtered_pos_x.push_back(subsonus_track_packet.corrected_position[0]);
			                median_pos_x = subsonus_track_packet.corrected_position[0];
			              }
			              else{
			                filtered_pos_x.erase(filtered_pos_x.begin());
			                filtered_pos_x.push_back(subsonus_track_packet.corrected_position[0]);
			                std::vector<double> sorted_filtered_pos_x = filtered_pos_x;
			                std::sort(sorted_filtered_pos_x.begin(), sorted_filtered_pos_x.end());
			                median_pos_x = sorted_filtered_pos_x[1];
			              }

			              if (filtered_pos_y.size() < maxSize){
			                filtered_pos_y.push_back(subsonus_track_packet.corrected_position[1]);
			                median_pos_y = subsonus_track_packet.corrected_position[1];
			              }
			              else{
			                filtered_pos_y.erase(filtered_pos_y.begin());
			                filtered_pos_y.push_back(subsonus_track_packet.corrected_position[1]);
			                std::vector<double> sorted_filtered_pos_y = filtered_pos_y;
			                std::sort(sorted_filtered_pos_y.begin(), sorted_filtered_pos_y.end());
			                median_pos_y = sorted_filtered_pos_y[1];
			              }

			              if (filtered_pos_z.size() < maxSize){
			                filtered_pos_z.push_back(subsonus_track_packet.corrected_position[2]);
			                median_pos_z = subsonus_track_packet.corrected_position[2];
			              }
			              else{
			                filtered_pos_z.erase(filtered_pos_z.begin());
			                filtered_pos_z.push_back(subsonus_track_packet.corrected_position[2]);
			                std::vector<double> sorted_filtered_pos_z = filtered_pos_z;
			                std::sort(sorted_filtered_pos_z.begin(), sorted_filtered_pos_z.end());
			                median_pos_z = sorted_filtered_pos_z[1];
			              }


			              if (filtered_range.size() < maxSize){
			                filtered_range.push_back(subsonus_track_packet.range);
			                median_range = subsonus_track_packet.range;
			              }
			              else{
			                filtered_range.erase(filtered_range.begin());
			                filtered_range.push_back(subsonus_track_packet.range);
			                std::vector<double> sorted_filtered_range = filtered_range;
			                std::sort(sorted_filtered_range.begin(), sorted_filtered_range.end());
			                median_range = sorted_filtered_range[1];
			              }

			              if (filtered_azimuth.size() < maxSize){
			                filtered_azimuth.push_back(subsonus_track_packet.azimuth*RADIANS_TO_DEGREES);
			                median_azimuth = subsonus_track_packet.azimuth*RADIANS_TO_DEGREES;
			              }
			              else{
			                filtered_azimuth.erase(filtered_azimuth.begin());
			                filtered_azimuth.push_back(subsonus_track_packet.azimuth*RADIANS_TO_DEGREES);
			                std::vector<double> sorted_filtered_azimuth = filtered_azimuth;
			                std::sort(sorted_filtered_azimuth.begin(), sorted_filtered_azimuth.end());
			                median_azimuth = sorted_filtered_azimuth[1];
			              }

			              msg_track_packet_filtered.remote_position_x = median_pos_x;
			              msg_track_packet_filtered.remote_position_y = median_pos_y;
			              msg_track_packet_filtered.remote_position_z = median_pos_z;
			              msg_track_packet_filtered.remote_range = median_range;
			              msg_track_packet_filtered.remote_azimuth = median_azimuth;

			              publisher_track_packet_filtered->publish(msg_track_packet_filtered);
						}
					}

					/* Ensure that you free the an_packet when your done with it or you will leak memory */
					an_packet_free(&an_packet);
				}
			}
		}
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<subsonus_pkg::msg::SubsonusRemoteTrackPacket>::SharedPtr publisher_track_packet;
  rclcpp::Publisher<subsonus_pkg::msg::SubsonusRemoteTrackPacket>::SharedPtr publisher_track_packet_filtered;

  size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NodeData>());
  rclcpp::shutdown();
  return 0;
}