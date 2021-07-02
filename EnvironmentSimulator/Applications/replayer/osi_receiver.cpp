/*
 * esmini - Environment Simulator Minimalistic
 * https://github.com/esmini/esmini
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) partners of Simulation Scenarios
 * https://sites.google.com/view/simulationscenarios
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "osi_common.pb.h"
#include "osi_object.pb.h"
#include "osi_sensorview.pb.h"
#include "osi_version.pb.h"
#include <signal.h>


#ifndef _WINDOWS
    #include <unistd.h>
    #define Sleep(x) usleep((x)*1000)
#endif

static bool quit;

#ifdef _WIN32
	#include <winsock2.h>
	#include <Ws2tcpip.h>
#else
	 /* Assume that any non-Windows platform uses POSIX-style sockets instead. */
	#include <sys/socket.h>
	#include <arpa/inet.h>
	#include <netdb.h>  /* Needed for getaddrinfo() and freeaddrinfo() */
	#include <unistd.h> /* Needed for close() */
#endif

#define OSI_OUT_PORT 48198
#define ES_SERV_TIMEOUT 500
#define MAX_MSG_SIZE 1024000
#define OSI_MAX_UDP_DATA_SIZE 8200

void CloseGracefully(int socket)
{
#ifdef _WIN32
	if (closesocket(socket) == SOCKET_ERROR)
#else
	if (close(socket) < 0)
#endif
	{
		printf("Failed closing socket");
	}

#ifdef _WIN32
	WSACleanup();
#endif
}

static void signal_handler(int s)
{
	printf("Caught signal %d - quit\n", s);

	quit = true;
}

int main(int argc, char* argv[])
{
	static int sock;
	struct sockaddr_in server_addr;
	struct sockaddr_in sender_addr;
	static unsigned short int iPortIn = OSI_OUT_PORT;   // Port for incoming packages
	static char large_buf[MAX_MSG_SIZE];
	socklen_t sender_addr_size = sizeof(sender_addr);
	struct timeval tv;

	// This struct must match the sender side
	struct {
		int counter;
		unsigned int datasize;
		char data[OSI_MAX_UDP_DATA_SIZE];
	} buf;

	quit = false;

	// Setup signal handler to catch Ctrl-C
	signal(SIGINT, signal_handler);

#ifdef _WIN32
	WSADATA wsa_data;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsa_data);
	if (iResult != NO_ERROR)
	{
		wprintf(L"WSAStartup failed with error %d\n", iResult);
		return -1;
	}
#endif

	sock = (int)socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0)
	{
		printf("socket failed\n");
		return -1;
	}

	//set timer for receive operations
	tv.tv_sec = 0;
	tv.tv_usec = ES_SERV_TIMEOUT;
#ifdef _WIN32
	int timeout_msec = 1000 * tv.tv_sec + tv.tv_usec;
	if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_msec, sizeof(timeout_msec)) < 0)
#else
	if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (struct timeval*)&tv, sizeof(tv)) == 0)
#endif
	{
		printf("socket SO_RCVTIMEO (receive timeout) not supported on this platform\n");
	}

	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(iPortIn);
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0)
	{
		printf("Bind failed");
		CloseGracefully(sock);
		return -1;
	}

	printf("Socket open. Waiting for OSI messages on port %d. Press Ctrl-C to quit.\n", OSI_OUT_PORT);

	osi3::GroundTruth gt;

	while (!quit)
	{
		// Fetch and parse OSI message
		buf.counter = 0;
		int retval;
		int receivedDataBytes = 0;
		while (buf.counter != -1)
		{
			retval = recvfrom(sock, (char*)&buf, sizeof(buf), 0, (struct sockaddr*)&sender_addr, &sender_addr_size);
			if (retval > 0)
			{
				if (buf.counter == 0)
				{
					// New message
					receivedDataBytes = 0;
				}
				memcpy(&large_buf[receivedDataBytes], buf.data, buf.datasize);
				receivedDataBytes += buf.datasize;
			}
		}

		if (retval > 0)
		{
			gt.ParseFromArray(large_buf, receivedDataBytes);

			// Print timestamp
			printf("timestamp: %.2f\n", gt.mutable_timestamp()->seconds() +
				1E-9 * gt.mutable_timestamp()->nanos());

			// Print object id, position, orientation and velocity
			for (int i = 0; i < gt.mutable_moving_object()->size(); i++)
			{
				printf(" obj id %d pos (%.2f, %.2f, %.2f) orientation (%.2f, %.2f, %.2f) velocity (%.2f, %.2f, %.2f) \n",
					(int)gt.mutable_moving_object(i)->mutable_id()->value(),
					gt.mutable_moving_object(i)->mutable_base()->mutable_position()->x(),
					gt.mutable_moving_object(i)->mutable_base()->mutable_position()->y(),
					gt.mutable_moving_object(i)->mutable_base()->mutable_position()->z(),
					gt.mutable_moving_object(i)->mutable_base()->mutable_orientation()->yaw(),
					gt.mutable_moving_object(i)->mutable_base()->mutable_orientation()->pitch(),
					gt.mutable_moving_object(i)->mutable_base()->mutable_orientation()->roll(),
					gt.mutable_moving_object(i)->mutable_base()->mutable_velocity()->x(),
					gt.mutable_moving_object(i)->mutable_base()->mutable_velocity()->y(),
					gt.mutable_moving_object(i)->mutable_base()->mutable_velocity()->z()
				);
			}
		}
		else
		{
			// No incoming messages, wait for a little while before polling again
			Sleep(10);
		}
	}

	return 0;
}
