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

#include "CommonMini.hpp"
#include "ScenarioGateway.hpp"
#include "Server.hpp"

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


using namespace scenarioengine;

#define DEFAULT_INPORT 48199 
#define ES_SERV_TIMEOUT 500

enum { SERV_NOT_STARTED, SERV_RUNNING, SERV_STOP, SERV_STOPPED };

static int state = SERV_NOT_STARTED;
static SE_Thread thread;
static SE_Mutex mutex;
static ScenarioGateway *scenarioGateway = 0;

namespace scenarioengine
{

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

		state = SERV_STOPPED;
	}

	void ServerThread(void *args)
	{
		static int sock;
		struct sockaddr_in server_addr;
		struct sockaddr_in sender_addr;
		static int iPortIn = DEFAULT_INPORT;   // Port for incoming packages
		EgoStateBuffer_t buf;
		socklen_t sender_addr_size = sizeof(sender_addr);

#ifdef _WIN32
		WSADATA wsa_data;
		int iResult = WSAStartup(MAKEWORD(2, 2), &wsa_data);
		if (iResult != NO_ERROR)
		{
			wprintf(L"WSAStartup failed with error %d\n", iResult);
			return;
		}
#endif

		sock = (int)socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if (sock < 0)
		{
			printf("socket failed\n");
			return;
		}

		//set timer for recv_socket
		static int timeout = ES_SERV_TIMEOUT;
		setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));

		server_addr.sin_family = AF_INET;
		server_addr.sin_port = htons(iPortIn);
		server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

		if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) != 0)
		{
			printf("Bind failed");
			CloseGracefully(sock);
			return;
		}
		state = SERV_RUNNING;
		
		while (state == SERV_RUNNING)
		{
			int ret = recvfrom(sock, (char*)&buf, sizeof(EgoStateBuffer_t), 0, (struct sockaddr *)&sender_addr, &sender_addr_size);
			if (ret >= 0)
			{
				printf("Server: Received Ego pos (%.2f, %.2f, %.2f) rot: (%.2f, %.2f, %.2f) speed: %.2f (%.2f km/h) wheel_angle: %.2f (%.2f deg)\n",
					buf.x, buf.y, buf.z, buf.h, buf.p, buf.r, buf.speed, 3.6 * buf.speed, buf.wheel_angle, 180 * buf.wheel_angle / M_PI);

				// Update Ego state
				mutex.Lock();

				scenarioGateway->reportObject(ObjectState(0, "Ego", 0, 1, 0, buf.x, buf.y, buf.z, buf.h, buf.p, buf.r, buf.speed, buf.wheel_angle));

				mutex.Unlock();
			}

		}

		CloseGracefully(sock);
	}

	void StartServer(ScenarioEngine *scenarioEngine)
	{
		// Fetch ScenarioGateway 
		scenarioGateway = scenarioEngine->getScenarioGateway();

		thread.Start(ServerThread, NULL);
	}

	void StopServer()
	{
		// Flag time to stop
		if (state == SERV_RUNNING)
		{
			state = SERV_STOP;
		}
		else
		{
			state = SERV_STOPPED;
		}
		
		// Wait/block until UDP server closed gracefully
		while (state != SERV_STOPPED) SE_sleep(100);
	}
}
