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

#pragma once

#include <string>

 // UDP network includes
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

#define ESMINI_DEFAULT_INPORT 48199

class UDPBase
{
public:
	UDPBase(unsigned short int port);
	~UDPBase() { CloseGracefully(); }
	int Bind(struct sockaddr_in &addr);
	void CloseGracefully();

protected:
	unsigned short int port_;
	int sock_;
	struct sockaddr_in server_addr_;
	struct sockaddr_in sender_addr_;
	socklen_t sender_addr_size_;
};

class UDPServer : private UDPBase
{
public:
	UDPServer(unsigned short int port, unsigned int timeoutMs = 500);
	~UDPServer() {}
	int Receive(char* buf, unsigned int size);
	unsigned short GetPort() { return port_; }
	unsigned int GetTimeout() { return timeoutMs_; }

private:
	unsigned int timeoutMs_;
};

class UDPClient : private UDPBase
{
public:
	UDPClient(unsigned short int port, std::string ipAddress);
	~UDPClient() {}
	int Send(char* buf, unsigned int size);
	unsigned short GetPort() { return port_; }
	std::string GetIPAddress() { return ipAddress_; }

private:
	std::string ipAddress_;
};