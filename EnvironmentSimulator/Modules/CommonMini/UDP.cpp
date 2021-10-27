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
#include "CommonMini.hpp"


UDPBase::UDPBase(unsigned short int port) : port_(port)
{
	sender_addr_size_ = sizeof(sender_addr_);

#ifdef _WIN32
	WSADATA wsa_data;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsa_data);
	if (iResult != NO_ERROR)
	{
		wprintf(L"WSAStartup failed with error %d\n", iResult);
		return;
	}
#endif

	if ((sock_ = (int)socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0)
	{
		LOG_AND_QUIT("socket failed");
		return;
	}
}

int UDPBase::Bind(struct sockaddr_in& addr)
{
	int retval = bind(sock_, (struct sockaddr*)&addr, sizeof(addr));
	if (retval != 0)
	{
#ifdef _WIN32
		LOG("bind error %d", WSAGetLastError());
#else
		perror("bind socket");
		LOG("bind socket error");
#endif
		CloseGracefully();
		LOG_AND_QUIT("Bind UDP socket on port %d failed (return code %d)", port_, retval);
		return -1;
	}

	return 0;
}

void UDPBase::CloseGracefully()
{
#ifdef _WIN32
	if (closesocket(sock_) == SOCKET_ERROR)
#else
	if (close(sock_) < 0)
#endif
	{
#ifdef _WIN32
		LOG("Failed closing socket %d", WSAGetLastError());
#else
		perror("close socket");
		LOG("close socket error");
#endif
	}

#ifdef _WIN32
	WSACleanup();
#endif
}

UDPServer::UDPServer(unsigned short int port, unsigned int timeoutMs) :
	timeoutMs_(timeoutMs), UDPBase(port)
{
	//set timer for receive operations
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = timeoutMs_;

#ifdef _WIN32
	int timeout_msec = 1000 * tv.tv_sec + tv.tv_usec;
	if (setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_msec, sizeof(timeout_msec)) != 0)
#else
	if (setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, (struct timeval*)&tv, sizeof(tv)) < 0)
#endif
	{
		printf("socket SO_RCVTIMEO (receive timeout) not supported on this platform\n");
	}

	// Prepare the sockaddr_in structure
	server_addr_.sin_family = AF_INET;
	server_addr_.sin_port = htons(port_);
	server_addr_.sin_addr.s_addr = htonl(INADDR_ANY);

	Bind(server_addr_);
}

int UDPServer::Receive(char* buf, unsigned int size)
{
	return recvfrom(sock_, buf, size, 0, (struct sockaddr*)&sender_addr_, &sender_addr_size_);
}

UDPClient::UDPClient(unsigned short int port, std::string ipAddress) :
	ipAddress_(ipAddress), UDPBase(port)
{
	// Prepare the sockaddr_in structure
	memset((char*)&server_addr_, 0, sizeof(server_addr_));
	server_addr_.sin_family = AF_INET;
	server_addr_.sin_port = htons(port_);
	inet_pton(AF_INET, ipAddress.c_str(), &server_addr_.sin_addr.s_addr);
}


int UDPClient::Send(char* buf, unsigned int size)
{
	return sendto(sock_, buf, size, 0, (struct sockaddr*)&server_addr_, sizeof(server_addr_));
}