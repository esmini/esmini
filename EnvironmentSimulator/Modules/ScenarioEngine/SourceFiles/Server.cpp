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

// #define SWAP_BYTE_ORDER_ESMINI  // Set when Ego state is sent from non Intel platforms, such as dSPACE

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
	}

	void ServerThread(void *args)
	{
		static int sock;
		struct sockaddr_in server_addr;
		struct sockaddr_in sender_addr;
		static unsigned short int iPortIn = DEFAULT_INPORT;   // Port for incoming packages
		EgoStateBuffer_t buf;
		socklen_t sender_addr_size = sizeof(sender_addr);
		struct timeval tv;

		state = SERV_NOT_STARTED;

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

		//set timer for receive operations
		tv.tv_sec = 0;
		tv.tv_usec = ES_SERV_TIMEOUT;
#ifdef _WIN32
		int timeout_msec = 1000 * tv.tv_sec + tv.tv_usec;
		if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_msec, sizeof(timeout_msec)) == 0)
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
			return;
		}
		state = SERV_RUNNING;

		double x_old = 0.0;
		double y_old = 0.0;
		double wheel_rot = 0.0;


		while (state == SERV_RUNNING)
		{
			int ret = recvfrom(sock, (char*)&buf, sizeof(EgoStateBuffer_t), 0, (struct sockaddr *)&sender_addr, &sender_addr_size);

#ifdef SWAP_BYTE_ORDER_ESMINI
			SwapByteOrder((unsigned char*)&buf, 4, sizeof(buf));
#endif

			// Find out wheel rotation from x, y displacement
			double ds = GetLengthOfLine2D(buf.x, buf.y, x_old, y_old);
			wheel_rot += SIGN(buf.speed) * fmod(ds / 0.35, 2 * M_PI); // wheel radius = 0.35 m
			x_old = buf.x;
			y_old = buf.y;

			if (ret >= 0)
			{
				printf("Server: Received Ego pos (%.2f, %.2f, %.2f) rot: (%.2f, %.2f, %.2f) speed: %.2f (%.2f km/h) wheel_angle: %.2f (%.2f deg)\n",
					buf.x, buf.y, buf.z, buf.h, buf.p, buf.r, buf.speed, 3.6 * buf.speed, buf.wheel_angle, 180 * buf.wheel_angle / M_PI);

				// Update Ego state
				mutex.Lock();

				OSCBoundingBox bbox = {0, 0, 0, 0, 0, 0}; // dummy bariable just to feed into the function

				scenarioGateway->reportObject(0, "Ego", static_cast<int>(Object::Type::VEHICLE), static_cast<int>(Vehicle::Category::CAR),0, 1, bbox,0, buf.speed, buf.wheel_angle, wheel_rot,  buf.x, buf.y, buf.z, buf.h, buf.p, buf.r);

				mutex.Unlock();
			}

		}

		CloseGracefully(sock);

		state = SERV_STOPPED;
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
		thread.Wait();
	}
}
