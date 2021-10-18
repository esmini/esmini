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

#include "UDP.hpp"

UDPServer::UDPServer(unsigned short int port) : port_(port), timeoutMs_(500)
{
	sender_addr_size_ = sizeof(sender_addr_);
	struct timeval tv;

#ifdef _WIN32
	WSADATA wsa_data;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsa_data);
	if (iResult != NO_ERROR)
	{
		wprintf(L"WSAStartup failed with error %d\n", iResult);
		return;
	}
#endif

	sock_ = (int)socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (sock_ < 0)
	{
		printf("socket failed\n");
		return;
	}

	//set timer for receive operations
	tv.tv_sec = 0;
	tv.tv_usec = timeoutMs_;
#ifdef _WIN32
	int timeout_msec = 1000 * tv.tv_sec + tv.tv_usec;
	if (setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_msec, sizeof(timeout_msec)) == 0)
#else
	if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (struct timeval*)&tv, sizeof(tv)) == 0)
#endif
	{
		printf("socket SO_RCVTIMEO (receive timeout) not supported on this platform\n");
	}

	server_addr_.sin_family = AF_INET;
	server_addr_.sin_port = htons(port_);
	server_addr_.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(sock_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) != 0)
	{
		printf("Bind UDP socket failed");
		CloseGracefully();
		return;
	}
}

void UDPServer::CloseGracefully()
{
#ifdef _WIN32
	if (closesocket(sock_) == SOCKET_ERROR)
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

int UDPServer::Receive(char* buf, unsigned int size)
{
	return recvfrom(sock_, buf, size, 0, (struct sockaddr*)&sender_addr_, &sender_addr_size_);
}